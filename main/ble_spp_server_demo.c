#include "ble_spp_server_demo.h"

static const adc_unit_t unit = ADC_UNIT_1;
static esp_adc_cal_characteristics_t *adc_chars;
static const adc1_channel_t channel = ADC1_CHANNEL_0;     //GPIO32 if ADC1
static const adc_atten_t atten = ADC_ATTEN_DB_11;

void adc_init();
void custom_gpio_config();
esp_timer_handle_t periodic_timer;

void xyz2rgb(float x, float y, float z, float *r, float *g, float *b);
void matMult(float m2[3], float m1[3][3], float r[3]);

static const uint8_t spp_adv_data[20] = {
    /* Flags */
    0x02,0x01,0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03,0x03,0xF0,0xAB,
    /* Complete Local Name in advertising */
    0x0B,0x09, 'S', 'M', 'A', 'R', 'T', '_', 'G', 'L', 'O', 'V', 'E'
};

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_uart_queue = NULL;
static xQueueHandle cmd_cmd_queue = NULL;

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node{
    int32_t len;
    uint8_t * node_buff;
    struct spp_receive_data_node * next_node;
}spp_receive_data_node_t;

static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t * first_node;
}spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    //SPP -  Service Declaration
    [SPP_IDX_SVC]                      	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    //SPP -  data receive characteristic Declaration
    [SPP_IDX_SPP_DATA_RECV_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  data receive characteristic Value
    [SPP_IDX_SPP_DATA_RECV_VAL]             	=
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN,sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    //SPP -  data notify characteristic Declaration
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  data notify characteristic Value
    [SPP_IDX_SPP_DATA_NTY_VAL]   =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_DATA_NTF_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    //SPP -  command characteristic Declaration
    [SPP_IDX_SPP_COMMAND_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    //SPP -  command characteristic Value
    [SPP_IDX_SPP_COMMAND_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN,sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    //SPP -  status characteristic Declaration
    [SPP_IDX_SPP_STATUS_CHAR]            =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    //SPP -  status characteristic Value
    [SPP_IDX_SPP_STATUS_VAL]                 =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN,sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    //SPP -  status characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_SPP_STATUS_CFG]         =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t),sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},
};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for(int i = 0; i < SPP_IDX_NB ; i++){
        if(handle == spp_handle_table[i]){
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if(temp_spp_recv_data_node_p1 == NULL){
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if(temp_spp_recv_data_node_p2 != NULL){
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff,p_data->write.value,p_data->write.len);
    if(SppRecvDataBuff.node_num == 0){
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }else{
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void print_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while(temp_spp_recv_data_node_p1 != NULL){
        uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}

uint16_t weight =0;
bno055_euler_t euler;

void uart_task(void *pvParameters)
{

    printf("\n task created\n");
    for (;;) {
    	if(is_connected)
    	{
        	vTaskDelay(100 / portTICK_PERIOD_MS);

    	}
    	else
    	{
            //Waiting for UART event.
            vTaskDelay(100 / portTICK_PERIOD_MS);
    	}

    }
    vTaskDelete(NULL);
}

static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10,&spp_uart_queue,0);
    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 2048, (void*)UART_NUM_0, 8, NULL);
}

void spp_cmd_task(void * arg)
{
    uint8_t * cmd_id;

    for(;;){
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if(xQueueReceive(cmd_cmd_queue, &cmd_id, portMAX_DELAY)) { //앱에서 ABF3에서 보내면 내용은 cmd_id에 있음
            esp_log_buffer_char("ABF3 COMMAND",(char *)(cmd_id),strlen((char *)cmd_id));
            free(cmd_id);
        }
    }
    vTaskDelete(NULL);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
    uint8_t res = 0xff;

    //ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n",event);
    switch (event) {
    	case ESP_GATTS_REG_EVT:
    	    ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        	ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        	esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
       	break;
    	case ESP_GATTS_READ_EVT:
            res = find_char_and_desr_index(p_data->read.handle);
            if(res == SPP_IDX_SPP_STATUS_VAL){
                //TODO:client read the status characteristic
            }
       	 break;
    	case ESP_GATTS_WRITE_EVT: {
    	    res = find_char_and_desr_index(p_data->write.handle);
            if(p_data->write.is_prep == false){
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
                if(res == SPP_IDX_SPP_COMMAND_VAL){
                    uint8_t * spp_cmd_buff = NULL;
                    spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                    if(spp_cmd_buff == NULL){
                        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                        break;
                    }
                    memset(spp_cmd_buff,0x0,(spp_mtu_size - 3));
                    memcpy(spp_cmd_buff,p_data->write.value,p_data->write.len);
                    xQueueSend(cmd_cmd_queue,&spp_cmd_buff,10/portTICK_PERIOD_MS);
                }else if(res == SPP_IDX_SPP_DATA_NTF_CFG){
                    if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = true;
                    }else if((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)){
                        enable_data_ntf = false;
                    }
                }
                else if(res == SPP_IDX_SPP_DATA_RECV_VAL){
#ifdef SPP_DEBUG_MODE
                    esp_log_buffer_char(GATTS_TABLE_TAG,(char *)(p_data->write.value),p_data->write.len);
#else
                    //uart_write_bytes(UART_NUM_0, (char *)(p_data->write.value), p_data->write.len);
                    if((p_data->write.value[0] == 0xfa) && (p_data->write.len == 4))
                    {
                    	if(p_data->write.value[3] == 0xa9)
                    	{
                    		if(p_data->write.value[1] == 0xff)
                    		{
                    			printf("haptic\n");
                    		}
                    		else if(p_data->write.value[1] == 0x00)
                    		{
                    			printf("haptic\n");
                    		}
                    	}
                    }
#endif
                }else{
                    //TODO:
                }
            }else if((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL)){
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
                store_wr_buffer(p_data);
            }
      	 	break;
    	}
    	case ESP_GATTS_EXEC_WRITE_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
    	    if(p_data->exec_write.exec_write_flag){
    	        print_write_buffer();
    	        free_write_buffer();
    	    }
    	    break;
    	}
    	case ESP_GATTS_MTU_EVT:
    	    spp_mtu_size = p_data->mtu.mtu;
    	    break;
    	case ESP_GATTS_CONF_EVT:
    	    break;
    	case ESP_GATTS_UNREG_EVT:
        	break;
    	case ESP_GATTS_DELETE_EVT:
        	break;
    	case ESP_GATTS_START_EVT:
        	break;
    	case ESP_GATTS_STOP_EVT:
        	break;
    	case ESP_GATTS_CONNECT_EVT:
    	    spp_conn_id = p_data->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    is_connected = true;
    	    printf("connected\n");
    	    esp_timer_start_periodic(periodic_timer, 10000); //10ms
    	    memcpy(&spp_remote_bda,&p_data->connect.remote_bda,sizeof(esp_bd_addr_t));
        	break;
    	case ESP_GATTS_DISCONNECT_EVT:
    	    is_connected = false;
    	    enable_data_ntf = false;
    	    esp_timer_stop(periodic_timer); //10ms
    	    esp_ble_gap_start_advertising(&spp_adv_params);
    	    break;
    	case ESP_GATTS_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CANCEL_OPEN_EVT:
    	    break;
    	case ESP_GATTS_CLOSE_EVT:
    	    break;
    	case ESP_GATTS_LISTEN_EVT:
    	    break;
    	case ESP_GATTS_CONGEST_EVT:
    	    break;
    	case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
    	    ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n",param->add_attr_tab.num_handle);
    	    if (param->add_attr_tab.status != ESP_GATT_OK){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
    	    }
    	    else if (param->add_attr_tab.num_handle != SPP_IDX_NB){
    	        ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
    	    }
    	    else {
    	        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
    	        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
    	    }
    	    break;
    	}
    	default:
    	    break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    //ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == spp_profile_tab[idx].gatts_if) {
                if (spp_profile_tab[idx].gatts_cb) {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch( wakeup_reason )
    {

        case ESP_SLEEP_WAKEUP_ALL: //!< Wakeup caused by external signal using RTC_IO
            ESP_LOGI(GATTS_TABLE_TAG, "Wakeup caused by external signal using RTC_IO");
            break;

        case ESP_SLEEP_WAKEUP_EXT0: //!< Wakeup caused by external signal using RTC_CNTL
            ESP_LOGI(GATTS_TABLE_TAG, "Wakeup caused by external signal using RTC_CNTL");
            break;

        case ESP_SLEEP_WAKEUP_EXT1: //!< Wakeup caused by timer
            ESP_LOGI(GATTS_TABLE_TAG, "Wakeup caused by timer");
            break;

        case ESP_SLEEP_WAKEUP_TOUCHPAD: //!< Wakeup caused by touchpad
            ESP_LOGI(GATTS_TABLE_TAG, "Wakeup caused by touchpad");
            break;

        case ESP_SLEEP_WAKEUP_ULP: //!< Wakeup caused by ULP program
            ESP_LOGI(GATTS_TABLE_TAG, "Wakeup caused by ULP program");
            break;

        case ESP_SLEEP_WAKEUP_GPIO: //!< Wakeup caused by GPIO (light sleep only)
            ESP_LOGI(GATTS_TABLE_TAG, "Wakeup caused by GPIO program");
            break;

        case ESP_SLEEP_WAKEUP_UART: //!< Wakeup caused by UART (light sleep only)
            ESP_LOGI(GATTS_TABLE_TAG, "Wakeup caused by UART program");
            break;

        default : //!< In case of deep sleep, reset was not caused by exit from deep sleep
            ESP_LOGI(GATTS_TABLE_TAG, "Wakeup was not caused by deep sleep (%d)",wakeup_reason);
            break;
    }
}


void SleepAndWakeup( void )
{
#if 0
    //ESP_ERROR_CHECK(esp_wifi_stop());

    gpio_set_direction(USB_DETECT, GPIO_MODE_DEF_INPUT); // USB
    gpio_set_direction(BUTTON_IO_NUM, GPIO_MODE_DEF_INPUT); // BTN
    gpio_set_pull_mode(USB_DETECT, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BUTTON_IO_NUM, GPIO_PULLDOWN_ONLY);

    gpio_set_level(USB_DETECT, 1);
    gpio_set_level(BUTTON_IO_NUM, 0);

//    esp_sleep_enable_ext0_wakeup(GPIO_NUM_18, 0); // BTN On
    esp_set_deep_sleep_wake_stub(&wake_stub);
    ESP_LOGI(TAG, "deep sleep start");
    esp_deep_sleep( 1 * 1000000);
#else
    esp_timer_stop(periodic_timer);

    custom_gpio_config();

    gpio_set_level(LED_R, 0);
    gpio_set_level(LED_G, 0);
    gpio_set_level(LED_B, 0);

    gpio_wakeup_enable(BUTTON_IO_NUM,GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(USB_DETECT,GPIO_INTR_LOW_LEVEL);
    /* Wake up in 2 seconds, or when button is pressed */
    //esp_sleep_enable_timer_wakeup(2000000);
    esp_sleep_enable_gpio_wakeup();

    //gpio_set_level(USB_DETECT, 1);
    //gpio_set_level(BUTTON_IO_NUM, 0);
    ESP_LOGI(GATTS_TABLE_TAG, "sleep start");
    vTaskDelay(pdMS_TO_TICKS(10));
    /* Enter sleep mode */
    esp_light_sleep_start();

    esp_timer_start_periodic(periodic_timer, 10000); //10ms
    ESP_LOGI(GATTS_TABLE_TAG, "Wakeup End");
    //esp_deep_sleep(1000);
#endif
}

static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(GATTS_TABLE_TAG,"eFuse Two Point: Supported\n");
    } else {
        ESP_LOGI(GATTS_TABLE_TAG,"eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(GATTS_TABLE_TAG,"eFuse Vref: Supported\n");
    } else {
        ESP_LOGI(GATTS_TABLE_TAG,"eFuse Vref: NOT supported\n");
    }
#endif
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(GATTS_TABLE_TAG,"Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(GATTS_TABLE_TAG,"Characterized using eFuse Vref\n");
    } else {
        ESP_LOGI(GATTS_TABLE_TAG,"Characterized using Default Vref\n");
    }
}

uint ms10_cnt = 0;
uint8_t tx_buf[14];
esp_err_t bno_err;
static void periodic_timer_callback(void* arg) //10ms_timer
{

	bno_err = bno055_read_euler_hrp(0, &euler);

	if( bno_err != ESP_OK ) {
		printf("bno055_get_quaternion() returned error: %02x \n", bno_err);
		exit(2);
	}
	//printf("%16lld\t%10d\t",time_mks, time_bno);
	//printf("%.6f\t%.6f\t%.6f\t%.6f\n", quat.w, quat.x, quat.y, quat.z );
	//printf("h: %3d r: %3d p: %3d\n",(euler.h/16),(euler.r/16),(euler.p/16));

	ms10_cnt++;
	if(ms10_cnt>9) // 100ms_timer
	{
		ms10_cnt = 0;

		tx_buf[0] = 0xfa;
		tx_buf[1] = (uint8_t)(voltage >> 8);
		tx_buf[2] = (uint8_t)(voltage);
		tx_buf[3] = (uint8_t)(weight >> 8);
		tx_buf[4] = (uint8_t)(weight);
		tx_buf[5] = (uint8_t)((euler.h/16)>>8);
		tx_buf[6] = (uint8_t)(euler.h/16);
		tx_buf[7] = (uint8_t)((euler.r/16)>>8);
		tx_buf[8] = (uint8_t)(euler.r/16);
		tx_buf[9] = (uint8_t)((euler.p/16)>>8);
		tx_buf[10] = (uint8_t)(euler.p/16);
		tx_buf[11] = (uint8_t) 0;
		tx_buf[12] = 0x55;
		tx_buf[13] = 0xa9;
		if(is_connected)
		{
			esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 14, tx_buf, false);
		}

	}
}

int btn_cnt = 0;
int btn_flag = 0;
uint adc_reading = 0;

RTC_DATA_ATTR int wakeup_cnt;
bno055_chip_info_t bno_info;
static const char *TAG = "bno055";

esp_timer_handle_t periodic_timer;
static i2c_dev_t i2c_switch = { 0 };

typedef struct
{
    i2c_dev_t i2c_dev;
}bno055_dev_t;

static bno055_dev_t sensors[SENSOR_COUNT] = { 0 };

esp_err_t bno055_init_desc(bno055_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = BNO055_ADDRESS_A;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = 1000000;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

void app_main(void)
{
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[13], PIN_FUNC_GPIO);
    esp_err_t ret;

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    wakeup_cnt++;
    ESP_LOGI(GATTS_TABLE_TAG, "wake up cnt %d",wakeup_cnt);
    print_wakeup_reason();

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    adc_init();
    custom_gpio_config();

    esp_err_t err;
    uint8_t reg_val;

    i2cdev_init();

    for (size_t i = 0; i < SENSOR_COUNT; i++)
    {
    	bno055_init_desc(&sensors[i],0,19,18);
    }

    tca9548_init_desc(&i2c_switch, 0x70, 0, 19, 18);
    for(uint8_t i=0;i<SENSOR_COUNT;i++)
    {
    	tca9548_set_channels(&i2c_switch, BIT(i));
        err = bno055_read_register(0, BNO055_CHIP_ID_ADDR, &reg_val);

        if( err == ESP_OK ) {

            ESP_LOGD(TAG, "BNO055 ID returned 0x%02X", reg_val);
            if( reg_val == BNO055_ID ) {
                ESP_LOGI(TAG, "BNO055 detected \n");
            }
            else {
                ESP_LOGE(TAG, "bno055_open() error: BNO055 NOT detected");
            }
        }
    }
#ifdef bno055_use1
    bno055_config_t bno_conf;
    i2c_number_t i2c_num = 0;

    esp_err_t err;
    err = bno055_set_default_conf(&bno_conf);
	bno_conf.sda_io_num = 19;
	bno_conf.scl_io_num = 18;
    err = bno055_open(i2c_num, &bno_conf);
    printf("bno055_open() returned 0x%02X \n", err);

    if( err != ESP_OK ) {
        printf("Program terminated!\n");
        err = bno055_close(i2c_num);
        printf("bno055_close() returned 0x%02X \n", err);
        exit(1);
    }

    err = bno055_set_opmode(i2c_num, OPERATION_MODE_NDOF);
    printf("bno055_set_opmode(OPERATION_MODE_NDOF) returned 0x%02x \n", err);
    vTaskDelay(500 / portTICK_RATE_MS);

    err = bno055_get_chip_info(i2c_num, &bno_info);
    bno055_displ_chip_info(bno_info);
#endif
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    spp_uart_init();

    cmd_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_cmd_task, "spp_cmd_task", 2048, NULL, 10, NULL);

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic",
    };

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    while (1)
     {
         //ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
         for (int i = 0; i < NO_OF_SAMPLES; i++)
         {
             if (unit == ADC_UNIT_1)
             {
            	 adc_reading += adc1_get_raw((adc1_channel_t)channel);
             }
         }
         adc_reading /= NO_OF_SAMPLES;
         //Convert adc_reading to voltage in mV
         voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars) * 3;
         //ESP_LOGI(TAG,"Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
         printf("CHRG: %d, %d, USB Detect: %c\n",gpio_get_level(CHRG), voltage, gpio_get_level(USB_DETECT)?'N':'Y');

		 if(gpio_get_level(BUTTON_IO_NUM) != 0)
		 {
			 btn_cnt++;
			 if(btn_cnt>3)
			 { // POwer off
				 btn_cnt = 0;
				 gpio_set_level(LED_R, 1);
				 gpio_set_level(LED_G, 1);
				 gpio_set_level(LED_B, 1);

				 while(gpio_get_level(BUTTON_IO_NUM))
				 {
					 vTaskDelay(pdMS_TO_TICKS(100));
				 }
				 SleepAndWakeup();
			 }
		 }

         if(gpio_get_level(USB_DETECT) == 0)
         { // USB on
             if(voltage > 4100)
             { // bat full
                 gpio_set_level(LED_R, 0);
                 gpio_set_level(LED_G, 1);
                 gpio_set_level(LED_B, 0);
             }
             else if(voltage < 3300)
             { // empty
                 gpio_set_level(LED_R, 1);
                 gpio_set_level(LED_G, 0);
                 gpio_set_level(LED_B, 0);
             }
             else
             { // charge
                 gpio_set_level(LED_R, 0);
                 gpio_set_level(LED_G, 1);
                 gpio_set_level(LED_B, 1);
             }
         }
         else
         { // battery work
             if(voltage < 3300)
             { // empty
                 gpio_set_level(LED_R, 1);
                 gpio_set_level(LED_G, 0);
                 gpio_set_level(LED_B, 0);
             }
             else
             { // run
                 gpio_set_level(LED_R, 0);
                 gpio_set_level(LED_G, 0);
                 gpio_set_level(LED_B, 1);
             }
         }
         vTaskDelay(pdMS_TO_TICKS(1000));
     }
    return;
}

void adc_init()
{
    //Configure ADC
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(channel, atten);
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

void custom_gpio_config()
{
    gpio_set_direction(LED_R, GPIO_MODE_DEF_OUTPUT); // LED_R
    gpio_set_direction(LED_G, GPIO_MODE_DEF_OUTPUT); // LED_G
    gpio_set_direction(LED_B, GPIO_MODE_DEF_OUTPUT); // LED_B
    gpio_set_direction(CHRG, GPIO_MODE_DEF_INPUT); // CHRG
    gpio_set_direction(USB_DETECT, GPIO_MODE_DEF_INPUT); // USB
    gpio_set_direction(BUTTON_IO_NUM, GPIO_MODE_DEF_INPUT); // BTN

    gpio_set_pull_mode(LED_R, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(LED_G, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(LED_B, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode(CHRG, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(USB_DETECT, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BUTTON_IO_NUM, GPIO_PULLDOWN_ONLY);
}

