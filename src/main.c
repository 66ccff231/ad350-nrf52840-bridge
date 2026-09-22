/*
 * XDS 功率计 → 标准 BLE 功率计 桥接固件
 * ======================================
 *
 * 数据流：
 *   XDS 功率计 --BLE 0x2A63--> 本机（Central） --BLE 0x1828 CPS--> 码表 / 行者 app
 *
 * 本机同时扮演两个角色：
 *   1. Central：扫描并连接 XDS 功率计，订阅 0x2A63 拿测量数据
 *   2. Peripheral：以标准 Cycling Power 服务（0x1828）广播，供码表连接
 *
 * 为什么不做 ANT+：
 *   Garmin/ANT 许可明确禁止在非 Nordic 原厂 nRF52 芯片上使用 ANT SoftDevice
 *   （S212/S312/S332/S340），而第三方 ProMicro 板普遍不在许可范围内。
 *   行者小G+ 第三代官方规格支持「ANT+/蓝牙双连接，功率计均可连」，
 *   所以走标准 BLE 即可满足需求，且完全避开许可问题。
 *
 * ⚠️ 开发状态说明（请如实看待）：
 *   本文件尚未在真实硬件上编译与运行过。写它时依赖的是 Zephyr 公开 API 的
 *   既有知识，作者环境无法编译验证。首次编译大概率需要小幅调整（见 README）。
 *   协议解析部分（xds_protocol.c）的字段偏移已用实测数据验证过。
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

#include "xds_protocol.h"

LOG_MODULE_REGISTER(xds_bridge, LOG_LEVEL_INF);

/* ------------------------------------------------------------------ */
/* 配置                                                                */
/* ------------------------------------------------------------------ */

/** 标准 Cycling Power 服务（用于扫描判定） */
#define XDS_SERVICE_UUID16      0x1828
/** Cycling Power Measurement：功率数据在这里 */
#define XDS_MEAS_CHAR_UUID16    0x2A63
/** Cycling Power Control Point（0x2A55）：xds_forwarding 会写启动命令
 *  {02 16 AA 10} 到这里。实测本设备返回 0x81 拒绝，且不发也能收到测量数据，
 *  所以本固件不实现这一步；若日后需要，见 README「已知未完成项」。 */
#define XDS_CTRL_CHAR_UUID16    0x2A55

/** 本机广播名（码表/手机会看到这个名字） */
#define DEVICE_NAME "XDS Power Bridge"

/** 数据超时：超过这么久没收到测量就认为掉线 */
#define DATA_TIMEOUT_MS 10000

/* ------------------------------------------------------------------ */
/* 共享状态                                                            */
/* ------------------------------------------------------------------ */

static struct k_mutex state_lock;
static xds_power_measurement_t latest;
static uint32_t last_rx_ms;
static bool have_data;
static atomic_t notify_pending = ATOMIC_INIT(0);

static struct bt_conn *sensor_conn;    /* 到 XDS 功率计的连接 */
static struct bt_conn *client_conn;    /* 码表连进来的连接 */
static atomic_t ccc_enabled = ATOMIC_INIT(0);

static bt_addr_le_t pending_addr;      /* 待连接的功率计地址 */
static bool scan_connecting;           /* 是否正在「停扫描 -> 发起连接」流程中 */

/* ------------------------------------------------------------------ */
/* 标准 Cycling Power Measurement 负载构造                             */
/*   flags(u16) = 0x0000 表示只带必选的 instantaneous power（s16）      */
/*   这是兼容性最好的最简形式，码表与手机都能解析                        */
/* ------------------------------------------------------------------ */

static uint8_t measurement_buf[4];

static void build_measurement(uint16_t power_w)
{
    int16_t p = (int16_t)MIN(power_w, (uint16_t)INT16_MAX);
    sys_put_le16(0x0000U, &measurement_buf[0]);                 /* flags */
    sys_put_le16((uint16_t)p, &measurement_buf[2]);             /* power */
}

/* ------------------------------------------------------------------ */
/* GATT 服务：标准 Cycling Power（0x1828），只暴露测量特征与 CCCD      */
/* ------------------------------------------------------------------ */

static ssize_t read_measurement(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             measurement_buf, sizeof(measurement_buf));
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bool on = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&ccc_enabled, on ? 1 : 0);
    LOG_INF("客户端%s了通知", on ? "开启" : "关闭");
}

/* 用 128 位完整形式写标准 UUID，避免依赖 UUID 类型注册 */
#define BT_UUID_CPS_VAL \
    BT_UUID_128_ENCODE(0x00001828, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_CPM_VAL \
    BT_UUID_128_ENCODE(0x00002a63, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)

static struct bt_uuid_128 uuid_cps = BT_UUID_INIT_128(BT_UUID_CPS_VAL);
static struct bt_uuid_128 uuid_cpm = BT_UUID_INIT_128(BT_UUID_CPM_VAL);

static struct bt_gatt_attr cps_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&uuid_cps),
    BT_GATT_CHARACTERISTIC(&uuid_cpm.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_measurement, NULL, measurement_buf),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service cps_svc = BT_GATT_SERVICE(cps_attrs);

/* ------------------------------------------------------------------ */
/* 通知：有数据更新且客户端已订阅时，把测量值推给码表                  */
/* ------------------------------------------------------------------ */

static void notify_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (atomic_get(&ccc_enabled) == 0)
    {
        return;
    }

    uint16_t power = 0;
    bool fresh;

    k_mutex_lock(&state_lock, K_FOREVER);
    power = latest.total_power_w;
    fresh = have_data && ((k_uptime_get_32() - last_rx_ms) < DATA_TIMEOUT_MS);
    k_mutex_unlock(&state_lock);

    build_measurement(fresh ? power : 0U);

    int err = bt_gatt_notify(NULL, &cps_attrs[1], measurement_buf,
                             sizeof(measurement_buf));
    if ((err != 0) && (err != -ENOTCONN) && (err != -EAGAIN))
    {
        LOG_WRN("通知失败 (err %d)", err);
    }
}

K_WORK_DEFINE(notify_work, notify_work_handler);

/* ------------------------------------------------------------------ */
/* Central：解析功率计通知                                             */
/* ------------------------------------------------------------------ */

static uint8_t notify_func(struct bt_conn *conn,
                           struct bt_gatt_subscribe_params *params,
                           const void *data, uint16_t length)
{
    ARG_UNUSED(conn);

    if (data == NULL)
    {
        LOG_WRN("功率计通知订阅被移除");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    xds_power_measurement_t m;
    if (!xds_power_measurement_parse((uint8_t const *)data, length, &m))
    {
        LOG_WRN("忽略过短的测量（%u 字节）", (unsigned int)length);
        return BT_GATT_ITER_CONTINUE;
    }

    k_mutex_lock(&state_lock, K_FOREVER);
    latest = m;
    last_rx_ms = k_uptime_get_32();
    have_data = true;
    k_mutex_unlock(&state_lock);

    LOG_INF("收到 总功率=%u W 踏频=%u rpm 左=%d 右=%d 角度=%d err=%u",
            m.total_power_w, m.cadence_rpm, m.left_power_w,
            m.right_power_w, m.angle_deg, m.error_code);

    k_work_submit(&notify_work);
    return BT_GATT_ITER_CONTINUE;
}

static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_discover_params discover_params;

/* 依次发现：服务 -> 测量特征 -> CCCD */
static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (attr == NULL)
    {
        if (subscribe_params.value_handle == 0U)
        {
            LOG_ERR("没找到 0x2A63 特征，无法接收功率数据");
        }
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_PRIMARY)
    {
        /* 找到服务，接着找测量特征 */
        struct bt_gatt_service_val *svc = attr->user_data;
        discover_params.uuid = NULL;
        discover_params.start_handle = attr->handle + 1;
        discover_params.end_handle = svc->end_handle;
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
        bt_gatt_discover(conn, &discover_params);
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC)
    {
        struct bt_gatt_chrc *chrc = attr->user_data;
        if (bt_uuid_cmp(chrc->uuid, &uuid_cpm.uuid) == 0)
        {
            subscribe_params.value_handle = chrc->value_handle;
            /* 找这个特征的 CCCD */
            discover_params.uuid = BT_UUID_GATT_CCC;
            discover_params.start_handle = chrc->value_handle + 1;
            discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
            bt_gatt_discover(conn, &discover_params);
            return BT_GATT_ITER_STOP;
        }
        return BT_GATT_ITER_CONTINUE;
    }

    if (params->type == BT_GATT_DISCOVER_DESCRIPTOR)
    {
        subscribe_params.ccc_handle = attr->handle;
        subscribe_params.value = BT_GATT_CCC_NOTIFY;
        subscribe_params.notify = notify_func;
        int err = bt_gatt_subscribe(conn, &subscribe_params);
        if (err != 0)
        {
            LOG_ERR("订阅失败 (err %d)", err);
        }
        else
        {
            LOG_INF("已订阅 0x2A63，开始接收功率数据");
        }
        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_STOP;
}

static void start_discovery(struct bt_conn *conn)
{
    memset(&subscribe_params, 0, sizeof(subscribe_params));
    subscribe_params.value_handle = 0U;

    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;
    discover_params.uuid = &uuid_cps.uuid;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err != 0)
    {
        LOG_ERR("服务发现启动失败 (err %d)", err);
    }
}

/* ------------------------------------------------------------------ */
/* 连接回调                                                            */
/* ------------------------------------------------------------------ */

static void sensor_connected(struct bt_conn *conn, uint8_t err)
{
    if (err != 0)
    {
        LOG_ERR("连接功率计失败 (err %u)", err);
        scan_connecting = false;
        return;
    }
    scan_connecting = false;
    LOG_INF("已连接功率计");
    start_discovery(conn);
}

static void sensor_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    LOG_WRN("与功率计断开 (reason 0x%02x)，将重新扫描", reason);

    k_mutex_lock(&state_lock, K_FOREVER);
    have_data = false;
    latest = (xds_power_measurement_t){0};
    k_mutex_unlock(&state_lock);

    bt_conn_unref(sensor_conn);
    sensor_conn = NULL;
    scan_connecting = false;
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
};

/* 码表连进来 */
static void client_connected(struct bt_conn *conn, uint8_t err)
{
    if (err != 0)
    {
        return;
    }
    LOG_INF("码表/手机已连接");
    client_conn = bt_conn_ref(conn);
}

static void client_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(reason);
    if (client_conn != NULL)
    {
        bt_conn_unref(client_conn);
        client_conn = NULL;
    }
    atomic_set(&ccc_enabled, 0);
    LOG_INF("码表/手机已断开");
}

/* 连接回调：用同一组回调，按是哪个连接区分角色 */
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    if (conn == sensor_conn)
    {
        sensor_connected(conn, err);
    }
    else
    {
        client_connected(conn, err);
    }
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    if (conn == sensor_conn)
    {
        sensor_disconnected(conn, reason);
    }
    else
    {
        client_disconnected(conn, reason);
    }
}

/* ------------------------------------------------------------------ */
/* 扫描：找带 0x1828 服务的设备                                        */
/* ------------------------------------------------------------------ */

static bool adv_match(struct bt_data *data, void *user_data)
{
    bool *found = user_data;

    if ((data->type != BT_DATA_UUID16_ALL) &&
        (data->type != BT_DATA_UUID16_SOME))
    {
        return true;
    }

    for (size_t i = 0; (i + 1U) < data->data_len; i += 2U)
    {
        if (sys_get_le16(&data->data[i]) == XDS_SERVICE_UUID16)
        {
            *found = true;
            return false;
        }
    }
    return true;
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
                      struct net_buf_simple *buf)
{
    if ((sensor_conn != NULL) || scan_connecting)
    {
        return;   /* 已连上或正在连 */
    }

    bool found = false;
    bt_data_parse(buf, adv_match, &found);
    if (!found)
    {
        return;
    }

    LOG_INF("发现功率计 %s，停止扫描后连接", bt_addr_le_str(&info->addr));

    /* 先记下目标地址，等扫描真正停止的回调里再发起连接 ——
     * 直接在这里调 bt_conn_le_create 会撞上 "扫描仍活动" 而返回 -EAGAIN。 */
    memcpy(&pending_addr, &info->addr, sizeof(pending_addr));
    scan_connecting = true;
    bt_le_scan_stop();
}

static void scan_stopped_cb(void)
{
    if (!scan_connecting)
    {
        return;
    }

    int err = bt_conn_le_create(&pending_addr, BT_CONN_LE_CREATE_CONN,
                                BT_LE_CONN_PARAM_DEFAULT, &sensor_conn);
    if (err != 0)
    {
        LOG_ERR("发起连接失败 (err %d)", err);
        sensor_conn = NULL;
        scan_connecting = false;
    }
}

static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_recv,
    .timeout = NULL,
    .stopped = scan_stopped_cb,
};

/* ------------------------------------------------------------------ */
/* 广播参数与数据                                                      */
/*   关键：广播里必须带 0x1828 服务 UUID，码表按服务过滤才搜得到我们。 */
/* ------------------------------------------------------------------ */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(XDS_SERVICE_UUID16)),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, sizeof(DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param adv_param =
    BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN,
                         BT_GAP_ADV_FAST_INT_MIN_2,
                         BT_GAP_ADV_FAST_INT_MAX_2,
                         NULL);

/* ------------------------------------------------------------------ */
/* 主函数                                                              */
/* ------------------------------------------------------------------ */

int main(void)
{
    int err;

    k_mutex_init(&state_lock);
    build_measurement(0);

    err = bt_enable(NULL);
    if (err != 0)
    {
        LOG_ERR("蓝牙初始化失败 (err %d)", err);
        return err;
    }
    LOG_INF("蓝牙已就绪");

    bt_conn_cb_register(&conn_callbacks);
    bt_le_scan_cb_register(&scan_callbacks);

    /* 注册标准 CPS 服务，并开始广播 */
    err = bt_gatt_service_register(&cps_svc);
    if (err != 0)
    {
        LOG_ERR("注册 CPS 服务失败 (err %d)", err);
        return err;
    }

    /* 广播数据里必须带上 0x1828，否则码表按服务过滤时搜不到我们 */
    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err != 0)
    {
        LOG_ERR("启动广播失败 (err %d)", err);
        return err;
    }
    LOG_INF("已开始广播「%s」，服务 0x1828 —— 码表/手机可搜索连接", DEVICE_NAME);

    /* 持续扫描功率计 */
    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, NULL);
    if (err != 0)
    {
        LOG_ERR("启动扫描失败 (err %d)", err);
    }

    while (1)
    {
        k_sleep(K_SECONDS(1));

        /* 掉线保护：数据超时就断开重连，避免卡在死连接上 */
        k_mutex_lock(&state_lock, K_FOREVER);
        bool stale = have_data && ((k_uptime_get_32() - last_rx_ms) > DATA_TIMEOUT_MS);
        k_mutex_unlock(&state_lock);

        if (stale && (sensor_conn != NULL))
        {
            LOG_WRN("超过 %d ms 没收到数据，重连功率计", DATA_TIMEOUT_MS);
            bt_conn_disconnect(sensor_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }

        if ((sensor_conn == NULL) && !bt_le_scan_is_started())
        {
            bt_le_scan_start(BT_LE_SCAN_PASSIVE, NULL);
        }

        /* 没数据时也定期推 0，让码表知道桥还活着 */
        k_work_submit(&notify_work);
    }

    return 0;
}
