/*
 * XDS 功率计 → 标准 BLE 功率计 桥接固件
 * ======================================
 *
 * 数据流：
 *   XDS 功率计 --BLE 0x2A63--> 本机（Central） --BLE 0x1818 CPS--> 手机 app / 码表
 *
 * 本机同时扮演两个角色：
 *   1. Central：扫描并连接 XDS 功率计，订阅 0x2A63 拿测量数据
 *   2. Peripheral：以标准 Cycling Power 服务（0x1818）广播，供手机/码表连接
 *
 * 注意两个 UUID 用途不同，别混：
 *   - 对外广播/提供：0x1818（标准自行车功率服务）
 *   - 扫描功率计用：0x1828（实测 XDS 功率计自己用的非标准 UUID，
 *                      按标准它其实是 Mesh Proxy）
 *
 * 为什么不做 ANT+：
 *   ANT 需要 Nordic 的预编译 SoftDevice（nRF52840 对应 S340），
 *   它占用 0x0~0x305D0，会覆盖本板 nice!nano 的 UF2 引导器（0x0~0x26000），
 *   因此刷入必须用 SWD 编程器，之后也无法再拖拽 UF2 刷机。
 *   加上许可限制非 Nordic 原厂芯片、行者小G+ 第三代本身支持蓝牙功率计，
 *   所以走标准 BLE 即可满足需求。
 *
 * 实测状态：
 *   本固件已在 ProMicro nRF52840 上编译、烧录并运行验证 —— 能连上功率计、
 *   六个字段的解析全部用原始字节核对过（含「总功率 = 左 + 右」的自洽验证）、
 *   广播标准功率服务、配对成功、第三方 BLE 功率计 app 可读取。
 *   已知限制（行者 app 搜不到、ANT+ 不可行等）见 README。
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

#include "xds_protocol.h"

LOG_MODULE_REGISTER(xds_bridge, LOG_LEVEL_INF);

/* ------------------------------------------------------------------ */
/* 状态指示灯（板载红色 LED，P0.15，高电平点亮）                       */
/*   板子定义来自 Zephyr 的 boards/others/promicro_nrf52840：          */
/*     led0: led_0 { gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>; }           */
/*                                                                     */
/*   为什么要这个：脱离电脑使用时看不到日志，LED 是唯一的状态反馈。      */
/*     慢闪(1Hz)  正在扫描功率计                                        */
/*     快闪        已发现功率计，正在连接                                */
/*     常亮        已连上功率计，正在转发数据                            */
/*     很慢的呼吸  启动中 / 出错                                        */
/* ------------------------------------------------------------------ */

#define LED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

enum bridge_state
{
    ST_BOOT = 0,
    ST_SCANNING,
    ST_CONNECTING,
    ST_CONNECTED,
    ST_ERROR,
};

static atomic_t bridge_state = ATOMIC_INIT(ST_BOOT);

static void led_update(struct k_work *work)
{
    static uint32_t tick;
    tick++;

    int st = (int)atomic_get(&bridge_state);
    bool on;

    switch (st)
    {
    case ST_SCANNING:
        on = ((tick % 10U) < 2U);          /* 100ms x10 = 1 秒一次慢闪 */
        break;
    case ST_CONNECTING:
        on = ((tick % 2U) == 0U);          /* 5Hz 快闪 */
        break;
    case ST_CONNECTED:
        on = true;                         /* 常亮 */
        break;
    case ST_ERROR:
        on = ((tick % 4U) < 1U);           /* 急促短闪表示出错 */
        break;
    default:
        on = ((tick % 30U) < 15U);         /* 启动中：很慢的呼吸 */
        break;
    }

    (void)gpio_pin_set_dt(&status_led, on ? 1 : 0);
}

K_WORK_DEFINE(led_work, led_update);

static void led_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_work_submit(&led_work);
}

K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);

static void led_init(void)
{
    if (!gpio_is_ready_dt(&status_led))
    {
        LOG_WRN("状态 LED 未就绪，跳过");
        return;
    }
    (void)gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);
    k_timer_start(&led_timer, K_MSEC(100), K_MSEC(100));
}

#define SET_STATE(s) atomic_set(&bridge_state, (s))

/* ------------------------------------------------------------------ */
/* 配置                                                                */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* UUID 定义                                                           */
/*                                                                     */
/* ⚠️ 这里有两个不同的 UUID，别搞混：                                    */
/*   CPS_SERVICE_UUID16 = 0x1818 是**标准自行车功率服务**，             */
/*                        我们自己广播/提供的必须是它（手机/码表认它）  */
/*   XDS_ADV_UUID16     = 0x1828 是实测中 XDS 功率计广播里带的 UUID    */
/*                        （它按道理该用 0x1818，但实测就是 0x1828），  */
/*                        扫描时用它来认功率计                          */
/*                                                                     */
/* 早先版本把两者混用成 0x1828，结果本机把自己广播成了「蓝牙网格代理」  */
/* 服务，行者 app 扫功率服务自然找不到它。                              */
/* ------------------------------------------------------------------ */

/** 标准 Cycling Power 服务 —— 我们的 GATT 服务与广播都用这个 */
#define CPS_SERVICE_UUID16      0x1818
/** GAP Appearance：0x0484 = "Cycling: Power Sensor"。
 *  取自蓝牙标准 Assigned Numbers 的 Appearance 表；
 *  很多运动 app 按这个字段筛选功率计，不广播就可能搜不到。 */
#define CPS_APPEARANCE          0x0484
/** Cycling Power Sensor Location 特征的值：5 = Left Crank。
 *  XDS 这台是曲柄功率计，报「左曲柄」最贴切。 */
#define CPS_SENSOR_LOCATION     0x05
/** 实测 XDS 功率计广播里出现的 UUID，用于扫描匹配 */
#define XDS_ADV_UUID16          0x1828
/** Cycling Power Measurement：功率数据在这里 */
#define XDS_MEAS_CHAR_UUID16    0x2A63
/** Cycling Power Feature：部分 app 连接后会读它 */
#define CPS_FEATURE_CHAR_UUID16 0x2A65
/** Cycling Power Control Point（0x2A66）。xds_forwarding 会写启动命令
 *  {02 16 AA 10}，实测本设备返回 0x81 拒绝，且不发也能收到测量数据，
 *  所以本固件不实现这一步；若日后需要，见 README「已知未完成项」。 */
#define XDS_CTRL_CHAR_UUID16    0x2A55

/** 本机广播名（码表/手机会看到这个名字） */
#define DEVICE_NAME "XDS Power Bridge"

/** 数据超时：超过这么久没收到测量就认为掉线 */
#define DATA_TIMEOUT_MS 10000

/** 连接失败后的退避时间：避免每秒猛试，把连接对象和射频都耗掉 */
#define CONNECT_RETRY_BACKOFF_MS 5000

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
static bool scan_running;              /* 我们自己记录扫描状态。
                                        * Zephyr 4.1 没有公开的
                                        * bt_le_scan_is_started()，所以自己记。 */
static bool adv_running;               /* 广播是否开着（连接后会停，需自己重启） */
static uint32_t connect_retry_after_ms; /* 连接失败后的退避截止时刻 */

/* ------------------------------------------------------------------ */
/* 标准 Cycling Power Measurement 负载构造                             */
/*   flags bit5 (0x0020) = 带曲柄数据：累计圈数(u16) + 上次曲柄事件时间   */
/*   事件时间单位 1/1024 秒，16 位回绕（每 64 秒一圈）                   */
/*                                                                     */
/*   为什么要带曲柄数据：功率计只给瞬时踏频，不带圈数。而空转（无负载）   */
/*   时功率必然为 0，只有曲柄圈数会动 —— 有了它才能区分                  */
/*   「已连上但没负载」和「压根没连上」，验证和排错都靠它。              */
/* ------------------------------------------------------------------ */

#define CPS_FLAG_CRANK  0x0020U

static uint8_t measurement_buf[8];

static uint16_t crank_revs;        /* 累计曲柄圈数 */
static uint16_t crank_evt_time;    /* 上次曲柄事件时间，1/1024 秒 */
static uint32_t crank_last_ms;     /* 上次更新的时刻 */
static uint32_t crank_phase_ms;    /* 当前这一圈已经走了多少毫秒 */

/* 曲柄数据是 CPS 里唯一的踏频载体：协议没有单独的「踏频」字段，接收端
 * 一律用
 *      踏频 = Δ圈数 / Δ事件时间
 * 反推。所以事件时间**必须和圈数一一对应**。
 *
 * ⚠️ 曾经踩过的坑（实测 Wahoo 平均 60、峰值 254 rpm）：
 *   把事件时间按墙上时钟匀速推进（每次 +dt），圈数却按踏频积分攒够一圈才加。
 *   于是 85 rpm 时每秒圈数是 1,1,2,1,1,2…，事件时间却恒 +1024，
 *   接收端读到 60,60,120,60… 来回跳；
 *   而 update_crank() 由计通知和主循环各触发一次，两次挨得极近时 dt<1ms，
 *   (dt*1024)/1000 取整成 0 → Δ事件时间=0 而 Δ圈数=1 → 接收端除零 → 254 rpm。
 *
 * 正确做法：事件时间只在真正转满一圈时推进，且增量严格等于那一圈的周期。
 * 这样任意两次通知之间的 Δ圈数/Δ事件时间 都恒等于真实踏频，
 * 和通知的快慢、抖动彻底解耦。 */
static void update_crank(uint16_t cadence_rpm)
{
    uint32_t now = k_uptime_get_32();

    if (crank_last_ms == 0U)
    {
        crank_last_ms = now;
        return;
    }

    uint32_t dt = now - crank_last_ms;
    crank_last_ms = now;
    if (dt == 0U)
    {
        return;
    }

    /* 没人订阅时本函数根本不会被调用（notify_work_handler 提前返回），
     * 等新设备订阅上来 dt 可能已是几分钟。这种「断档」一律当成重新开始，
     * 否则会按上次的踏频一口气补出上千圈，累计值瞬间跳一大截。 */
    if (dt > 2000U)
    {
        crank_phase_ms = 0U;
        return;
    }

    if (cadence_rpm == 0U)
    {
        /* 停踩：圈内相位归零，事件时间原地不动。
         * 接收端看到 Δ圈数=0 且 Δ事件时间=0，就会把踏频判为 0。 */
        crank_phase_ms = 0U;
        return;
    }

    uint32_t period_ms = 60000U / cadence_rpm;      /* 转一圈要多少毫秒 */
    if (period_ms == 0U)
    {
        period_ms = 1U;
    }

    uint32_t evt_step = 61440U / cadence_rpm;       /* 60 * 1024 / rpm */
    if (evt_step == 0U)
    {
        evt_step = 1U;
    }

    crank_phase_ms += dt;
    while (crank_phase_ms >= period_ms)
    {
        crank_phase_ms -= period_ms;
        crank_revs++;
        crank_evt_time = (uint16_t)(crank_evt_time + evt_step);
    }
}

static void build_measurement(uint16_t power_w)
{
    int16_t p = (int16_t)MIN(power_w, (uint16_t)INT16_MAX);
    sys_put_le16(CPS_FLAG_CRANK, &measurement_buf[0]);          /* flags */
    sys_put_le16((uint16_t)p, &measurement_buf[2]);             /* 瞬时功率 */
    sys_put_le16(crank_revs, &measurement_buf[4]);              /* 累计圈数 */
    sys_put_le16(crank_evt_time, &measurement_buf[6]);          /* 事件时间 */
}

/* ------------------------------------------------------------------ */
/* GATT 服务：标准 Cycling Power（0x1818）                             */
/*   我们对外提供的服务必须是标准 0x1818 —— 手机/码表按它过滤。         */
/*   功率计自己用的 0x1828 只用于我们做 Central 时去发现它（见 uuid_xds）*/
/* ------------------------------------------------------------------ */

static ssize_t read_measurement(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             measurement_buf, sizeof(measurement_buf));
}

/** Cycling Power Feature：0 表示不支持可选特性。
 *  不少 app 连上后会读它，没有这个特征可能导致连接后被判为不兼容。 */
static ssize_t read_feature(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            void *buf, uint16_t len, uint16_t offset)
{
    static const uint8_t feature[4] = {0x00, 0x00, 0x00, 0x00};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, feature,
                             sizeof(feature));
}

/** Sensor Location：1 字节，说明功率计装在哪儿（5 = 左曲柄）。
 *  有些 app 连上后会读它。 */
static ssize_t read_sensor_location(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset)
{
    static const uint8_t loc = CPS_SENSOR_LOCATION;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &loc, sizeof(loc));
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bool on = (value == BT_GATT_CCC_NOTIFY);
    atomic_set(&ccc_enabled, on ? 1 : 0);
    LOG_INF("客户端%s了通知", on ? "开启" : "关闭");
}

/* 用 128 位完整形式写 UUID，避免依赖 UUID 类型注册。
 * 注意 0x1818（我们的服务）与 0x1828（功率计的服务）是两个不同的东西。 */
#define BT_UUID_CPS_VAL \
    BT_UUID_128_ENCODE(0x00001818, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_XDS_VAL \
    BT_UUID_128_ENCODE(0x00001828, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_CPM_VAL \
    BT_UUID_128_ENCODE(0x00002a63, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_CPF_VAL \
    BT_UUID_128_ENCODE(0x00002a65, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_CPL_VAL \
    BT_UUID_128_ENCODE(0x00002a5b, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)

static struct bt_uuid_128 uuid_cps = BT_UUID_INIT_128(BT_UUID_CPS_VAL);
static struct bt_uuid_128 uuid_xds = BT_UUID_INIT_128(BT_UUID_XDS_VAL);
static struct bt_uuid_128 uuid_cpm = BT_UUID_INIT_128(BT_UUID_CPM_VAL);
static struct bt_uuid_128 uuid_cpf = BT_UUID_INIT_128(BT_UUID_CPF_VAL);
static struct bt_uuid_128 uuid_cpl = BT_UUID_INIT_128(BT_UUID_CPL_VAL);

/* 注意下标：cps_attrs[1] 必须是测量特征（notify 时按句柄取它） */
static struct bt_gatt_attr cps_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&uuid_cps),
    BT_GATT_CHARACTERISTIC(&uuid_cpm.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_measurement, NULL, measurement_buf),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&uuid_cpf.uuid,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_feature, NULL, NULL),
    BT_GATT_CHARACTERISTIC(&uuid_cpl.uuid,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_sensor_location, NULL, NULL),
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
    uint16_t cadence = 0;
    bool fresh;

    k_mutex_lock(&state_lock, K_FOREVER);
    power = latest.total_power_w;
    cadence = latest.cadence_rpm;
    fresh = have_data && ((k_uptime_get_32() - last_rx_ms) < DATA_TIMEOUT_MS);
    k_mutex_unlock(&state_lock);

    /* 曲柄圈数要一直积分（即使功率为 0），否则空转时圈数不动就没法判断链路 */
    update_crank(fresh ? cadence : 0U);

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

    /* 同时打印原始字节。偏移 0-1 的「总功率」已在真实骑行负载下验证
     * （Wahoo 与官方 app 平均 63 W / 峰值 232 W 一致），留着他日核对字段。 */
    LOG_HEXDUMP_INF(data, length, "原始负载");

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
    /* 这里找的是**功率计**的服务，实测它是 0x1828（并非标准的 0x1818） */
    discover_params.uuid = &uuid_xds.uuid;

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

        /* 连接失败时本回调也会被调用，此时必须释放 bt_conn_le_create()
         * 给出的引用 —— 否则每失败一次漏一个连接对象，
         * CONFIG_BT_MAX_CONN 耗尽后整个蓝牙栈就废了（实测就是这样停摆的）。 */
        if (sensor_conn != NULL)
        {
            bt_conn_unref(sensor_conn);
            sensor_conn = NULL;
        }
        scan_connecting = false;
        connect_retry_after_ms = k_uptime_get_32() + CONNECT_RETRY_BACKOFF_MS;
        SET_STATE(ST_SCANNING);
        return;
    }
    scan_connecting = false;
    SET_STATE(ST_CONNECTED);
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

    /* 判空：失败路径上可能已经被清掉了 */
    if (sensor_conn != NULL)
    {
        bt_conn_unref(sensor_conn);
        sensor_conn = NULL;
    }
    scan_connecting = false;
    connect_retry_after_ms = k_uptime_get_32() + CONNECT_RETRY_BACKOFF_MS;
    SET_STATE(ST_SCANNING);
}

/* 前向声明：连接断开回调里要重启广播，而该函数定义在后面 */
static void start_advertising(void);

/* 码表连进来 */
static void client_connected(struct bt_conn *conn, uint8_t err)
{
    if (err != 0)
    {
        return;
    }
    LOG_INF("码表/手机已连接");
    client_conn = bt_conn_ref(conn);

    /* 可连接广播在建立连接后会自动停止，这里同步标志，
     * 让断开后（或主循环兜底）能重新开起来。 */
    adv_running = false;
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

    /* 关键：Zephyr 不会在断开后自动恢复广播。
     * 不重启的话，手机连过一次之后就永远搜不到本机了（实测就是这样）。 */
    start_advertising();
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

/* 回调函数都已定义，这里再填内容 */
static struct bt_conn_cb conn_callbacks = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
};

/* ------------------------------------------------------------------ */
/* 配对状态日志：手机/码表来配对时能看到究竟发生了什么                 */
/* ------------------------------------------------------------------ */

static void pairing_complete_cb(struct bt_conn *conn, bool bonded)
{
    ARG_UNUSED(conn);
    LOG_INF("配对完成（bonded=%d）", bonded);
}

static void pairing_failed_cb(struct bt_conn *conn, enum bt_security_err reason)
{
    ARG_UNUSED(conn);
    LOG_WRN("配对失败（reason=%d）", reason);
}

static struct bt_conn_auth_info_cb auth_info_callbacks = {
    .pairing_complete = pairing_complete_cb,
    .pairing_failed = pairing_failed_cb,
};

/* ------------------------------------------------------------------ */
/* 扫描：找广播里带 0x1818 或 0x1828 的设备（功率计实测是后者）        */
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
        uint16_t const u = sys_get_le16(&data->data[i]);
        /* 实测功率计广播的是 0x1828；为稳妥也接受标准的 0x1818 */
        if ((u == XDS_ADV_UUID16) || (u == CPS_SERVICE_UUID16))
        {
            *found = true;
            return false;
        }
    }
    return true;
}

/* 连接工作项：scan_recv 里要用到它，所以先声明并定义；
 * 处理函数本体写在 scan_recv 之后。 */
static void connect_work_handler(struct k_work *work);
K_WORK_DEFINE(connect_work, connect_work_handler);

/* 调试用：把扫描到的设备打出来，方便判断功率计到底有没有在广播。
 * 用一张小表记住已打印过的地址，避免同一设备刷屏。 */
#define SEEN_MAX 16
static bt_addr_le_t seen_addrs[SEEN_MAX];
static uint8_t seen_count;

static void log_seen_device(const struct bt_le_scan_recv_info *info, bool has_cps)
{
    for (uint8_t i = 0U; i < seen_count; i++)
    {
        if (bt_addr_le_cmp(info->addr, &seen_addrs[i]) == 0)
        {
            return;   /* 已经打过 */
        }
    }
    if (seen_count >= SEEN_MAX)
    {
        seen_count = 0U;   /* 表满就重新开始记 */
    }
    seen_addrs[seen_count] = *info->addr;
    seen_count++;

    LOG_INF("扫描到 %s  rssi %d dBm  %s", bt_addr_le_str(info->addr),
            info->rssi, has_cps ? "匹配功率计 <<<" : "不匹配");
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
                      struct net_buf_simple *buf)
{
    /* 先解析并记录：不管后续是否要连接，都想知道现场有哪些设备 */
    bool has_cps = false;
    bt_data_parse(buf, adv_match, &has_cps);
    log_seen_device(info, has_cps);

    if ((sensor_conn != NULL) || scan_connecting)
    {
        return;   /* 已连上或正在连 */
    }

    /* 连接失败后先退避，别一秒一次猛试 */
    if (k_uptime_get_32() < connect_retry_after_ms)
    {
        return;
    }

    /* 关键：本机自己也广播功率服务，不排除自己的地址就会「自己连自己」 */
    bt_addr_le_t own[CONFIG_BT_ID_MAX];
    size_t own_count = ARRAY_SIZE(own);
    bt_id_get(own, &own_count);
    for (size_t i = 0U; i < own_count; i++)
    {
        if (bt_addr_le_cmp(info->addr, &own[i]) == 0)
        {
            return;   /* 这是自己 */
        }
    }

    if (!has_cps)
    {
        return;   /* 不是功率计 */
    }

    LOG_INF("发现功率计 %s，停止扫描后连接", bt_addr_le_str(info->addr));
    SET_STATE(ST_CONNECTING);

    /* 不能在这里直接停止扫描并连接：
     * scan_recv 跑在蓝牙接收线程上，就地调 bt_le_scan_stop() 有死锁风险，
     * 而且扫描未真正停止时 bt_conn_le_create() 会返回 -EAGAIN。
     * 所以丢给系统工作队列去做。
     *
     * 注意 info->addr 是**指针**，要解引用再拷 —— 之前写成 &info->addr
     * 等于把指针本身的内存当地址用，连的自然是垃圾地址。 */
    pending_addr = *info->addr;
    scan_connecting = true;
    k_work_submit(&connect_work);
}

/* 在系统工作队列里：先停扫描，再发起连接 */
static void connect_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!scan_connecting)
    {
        return;
    }

    int err = bt_le_scan_stop();
    if ((err != 0) && (err != -EALREADY))
    {
        LOG_WRN("停止扫描失败 (err %d)", err);
    }
    scan_running = false;

    err = bt_conn_le_create(&pending_addr, BT_CONN_LE_CREATE_CONN,
                            BT_LE_CONN_PARAM_DEFAULT, &sensor_conn);
    if (err != 0)
    {
        LOG_ERR("发起连接失败 (err %d)", err);
        sensor_conn = NULL;
        scan_connecting = false;
    }
}

/* 注意：Zephyr 4.1 的 bt_le_scan_cb 只有 recv / timeout，没有 stopped 成员，
 * 所以「停扫描后再连接」改用上面的工作队列实现。 */
static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_recv,
};

/** @brief 开始扫描功率计，并记录状态（Zephyr 没有公开的「是否在扫描」查询接口） */
static void start_scan(void)
{
    if (scan_running)
    {
        return;
    }

    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, NULL);
    if (err == 0)
    {
        scan_running = true;
        SET_STATE(ST_SCANNING);
        LOG_INF("开始扫描功率计（广播 0x1818/0x1828 的设备）");
    }
    else if (err == -EALREADY)
    {
        scan_running = true;   /* 已经在扫了 */
    }
    else
    {
        LOG_ERR("启动扫描失败 (err %d)", err);
    }
}

/* ------------------------------------------------------------------ */
/* 广播参数与数据                                                      */
/*   两个要点：                                                        */
/*     1. 广播里必须带标准功率服务 0x1818，码表/app 按服务过滤才搜得到  */
/*     2. 设备名放进**广播包**而不是扫描响应 —— 有些手机 app 只在       */
/*        广播包里读名字，放在扫描响应里它会看不到这台设备              */
/* ------------------------------------------------------------------ */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    /* 对外广播必须是标准功率服务 0x1818，手机/码表按它过滤 */
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(CPS_SERVICE_UUID16)),
    /* Appearance 必须广播！0x0484 = "Cycling: Power Sensor"（小端存放）。
     * 很多运动 app 按 Appearance 筛选功率计，缺了这个字段就搜不到本机 ——
     * 行者 app 连不上大概率就是这个原因。 */
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
                  (CPS_APPEARANCE & 0xFF), (CPS_APPEARANCE >> 8)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, sizeof(DEVICE_NAME) - 1),
};
/* 3 + 4 + 4 + 17 = 28 字节，仍在 31 字节 legacy 广播上限内 */

static const struct bt_le_adv_param adv_param =
    BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN,
                         BT_GAP_ADV_FAST_INT_MIN_2,
                         BT_GAP_ADV_FAST_INT_MAX_2,
                         NULL);

/** @brief 开始（或重新开始）广播。
 *
 * 为什么需要重复调用：可连接广播在建立连接后会**自动停止**，
 * Zephyr 不会自己恢复。不手动重启的话，手机/码表连过一次之后
 * 就再也搜不到本机了 —— 这是实测踩到的坑。 */
static void start_advertising(void)
{
    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err == 0)
    {
        adv_running = true;
        LOG_INF("已开始广播「%s」，服务 0x1818（标准功率服务）", DEVICE_NAME);
    }
    else if (err == -EALREADY)
    {
        adv_running = true;   /* 已经在广播 */
    }
    else
    {
        LOG_ERR("启动广播失败 (err %d)", err);
    }
}

/* ------------------------------------------------------------------ */
/* 主函数                                                              */
/* ------------------------------------------------------------------ */

int main(void)
{
    int err;

    k_mutex_init(&state_lock);
    build_measurement(0);
    led_init();                 /* 先点亮状态灯，方便肉眼判断板子有没有起来 */

    err = bt_enable(NULL);
    if (err != 0)
    {
        LOG_ERR("蓝牙初始化失败 (err %d)", err);
        SET_STATE(ST_ERROR);
        return err;
    }
    LOG_INF("蓝牙已就绪");

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_info_cb_register(&auth_info_callbacks);
    bt_le_scan_cb_register(&scan_callbacks);

    /* 注册标准 CPS 服务，并开始广播 */
    err = bt_gatt_service_register(&cps_svc);
    if (err != 0)
    {
        LOG_ERR("注册 CPS 服务失败 (err %d)", err);
        SET_STATE(ST_ERROR);
        return err;
    }

    /* 广播数据里必须带上 0x1818，否则手机/码表按服务过滤时搜不到我们 */
    start_advertising();
    if (!adv_running)
    {
        SET_STATE(ST_ERROR);
        return -EIO;
    }

    /* 持续扫描功率计 */
    start_scan();

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

        /* 没在扫描也没连上，就重新开始找功率计 */
        if ((sensor_conn == NULL) && !scan_running && !scan_connecting)
        {
            start_scan();
        }

        /* 兜底：没有手机/码表连着却不在广播，就重新开广播。
         * （正常路径在 client_disconnected 里重启，这里防意外情况） */
        if (!adv_running && (client_conn == NULL))
        {
            LOG_WRN("广播不在运行，重新开启");
            start_advertising();
        }

        /* 没数据时也定期推 0，让码表知道桥还活着 */
        k_work_submit(&notify_work);
    }

    return 0;
}
