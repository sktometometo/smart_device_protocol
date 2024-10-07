#include "AtomSPK.h"

const double sinmap[] = {0.0,
                         0.01745240643728351,
                         0.03489949670250097,
                         0.05233595624294383,
                         0.0697564737441253,
                         0.08715574274765817,
                         0.10452846326765346,
                         0.12186934340514748,
                         0.13917310096006544,
                         0.15643446504023087,
                         0.17364817766693033,
                         0.1908089953765448,
                         0.20791169081775931,
                         0.224951054343865,
                         0.24192189559966773,
                         0.25881904510252074,
                         0.27563735581699916,
                         0.29237170472273677,
                         0.3090169943749474,
                         0.32556815445715664,
                         0.3420201433256687,
                         0.35836794954530027,
                         0.374606593415912,
                         0.3907311284892737,
                         0.40673664307580015,
                         0.42261826174069944,
                         0.4383711467890774,
                         0.45399049973954675,
                         0.4694715627858908,
                         0.48480962024633706,
                         0.49999999999999994,
                         0.5150380749100542,
                         0.5299192642332049,
                         0.5446390350150271,
                         0.5591929034707469,
                         0.573576436351046,
                         0.5877852522924731,
                         0.6018150231520483,
                         0.6156614753256582,
                         0.6293203910498374,
                         0.6427876096865393,
                         0.6560590289905072,
                         0.6691306063588582,
                         0.6819983600624985,
                         0.6946583704589973,
                         0.7071067811865476,
                         0.7193398003386511,
                         0.7313537016191705,
                         0.7431448254773941,
                         0.754709580222772,
                         0.766044443118978,
                         0.7771459614569708,
                         0.788010753606722,
                         0.7986355100472928,
                         0.8090169943749475,
                         0.8191520442889918,
                         0.8290375725550417,
                         0.8386705679454239,
                         0.8480480961564261,
                         0.8571673007021122,
                         0.8660254037844386,
                         0.8746197071393957,
                         0.8829475928589269,
                         0.8910065241883678,
                         0.898794046299167,
                         0.9063077870366499,
                         0.9135454576426009,
                         0.9205048534524403,
                         0.9271838545667874,
                         0.9335804264972017,
                         0.9396926207859083,
                         0.9455185755993167,
                         0.9510565162951535,
                         0.9563047559630354,
                         0.9612616959383189,
                         0.9659258262890683,
                         0.9702957262759965,
                         0.9743700647852352,
                         0.9781476007338056,
                         0.981627183447664,
                         0.984807753012208,
                         0.9876883405951378,
                         0.9902680687415703,
                         0.992546151641322,
                         0.9945218953682733,
                         0.9961946980917455,
                         0.9975640502598242,
                         0.9986295347545738,
                         0.9993908270190958,
                         0.9998476951563913,
                         1.0,
                         0.9998476951563913,
                         0.9993908270190958,
                         0.9986295347545738,
                         0.9975640502598242,
                         0.9961946980917455,
                         0.9945218953682734,
                         0.9925461516413221,
                         0.9902680687415704,
                         0.9876883405951377,
                         0.984807753012208,
                         0.981627183447664,
                         0.9781476007338057,
                         0.9743700647852352,
                         0.9702957262759965,
                         0.9659258262890683,
                         0.9612616959383189,
                         0.9563047559630355,
                         0.9510565162951536,
                         0.9455185755993168,
                         0.9396926207859084,
                         0.9335804264972017,
                         0.9271838545667874,
                         0.9205048534524404,
                         0.913545457642601,
                         0.90630778703665,
                         0.8987940462991669,
                         0.8910065241883679,
                         0.8829475928589271,
                         0.8746197071393959,
                         0.8660254037844387,
                         0.8571673007021123,
                         0.8480480961564261,
                         0.838670567945424,
                         0.8290375725550417,
                         0.819152044288992,
                         0.8090169943749475,
                         0.7986355100472927,
                         0.788010753606722,
                         0.777145961456971,
                         0.766044443118978,
                         0.7547095802227718,
                         0.7431448254773942,
                         0.7313537016191706,
                         0.7193398003386514,
                         0.7071067811865476,
                         0.6946583704589971,
                         0.6819983600624985,
                         0.6691306063588583,
                         0.6560590289905073,
                         0.6427876096865395,
                         0.6293203910498377,
                         0.6156614753256584,
                         0.6018150231520482,
                         0.5877852522924732,
                         0.5735764363510464,
                         0.5591929034707469,
                         0.5446390350150269,
                         0.5299192642332049,
                         0.5150380749100544,
                         0.49999999999999994,
                         0.48480962024633717,
                         0.4694715627858911,
                         0.45399049973954686,
                         0.4383711467890773,
                         0.4226182617406995,
                         0.40673664307580043,
                         0.39073112848927416,
                         0.37460659341591224,
                         0.3583679495453002,
                         0.3420201433256689,
                         0.32556815445715703,
                         0.3090169943749475,
                         0.29237170472273705,
                         0.27563735581699966,
                         0.258819045102521,
                         0.24192189559966773,
                         0.22495105434386478,
                         0.20791169081775931,
                         0.19080899537654497,
                         0.17364817766693028,
                         0.15643446504023098,
                         0.13917310096006574,
                         0.12186934340514755,
                         0.10452846326765373,
                         0.08715574274765864,
                         0.06975647374412552,
                         0.05233595624294381,
                         0.0348994967025007,
                         0.01745240643728344,
                         0.0,
                         -0.017452406437283192,
                         -0.0348994967025009,
                         -0.052335956242943564,
                         -0.06975647374412483,
                         -0.08715574274765794,
                         -0.10452846326765305,
                         -0.12186934340514774,
                         -0.13917310096006552,
                         -0.15643446504023073,
                         -0.17364817766693047,
                         -0.19080899537654472,
                         -0.20791169081775907,
                         -0.22495105434386498,
                         -0.2419218955996675,
                         -0.25881904510252035,
                         -0.275637355816999,
                         -0.2923717047227364,
                         -0.30901699437494773,
                         -0.32556815445715676,
                         -0.34202014332566866,
                         -0.35836794954530043,
                         -0.374606593415912,
                         -0.39073112848927355,
                         -0.4067366430757998,
                         -0.4226182617406993,
                         -0.43837114678907707,
                         -0.45399049973954625,
                         -0.46947156278589086,
                         -0.48480962024633695,
                         -0.5000000000000001,
                         -0.5150380749100542,
                         -0.5299192642332048,
                         -0.5446390350150271,
                         -0.5591929034707467,
                         -0.5735764363510458,
                         -0.587785252292473,
                         -0.601815023152048,
                         -0.6156614753256578,
                         -0.6293203910498376,
                         -0.6427876096865393,
                         -0.6560590289905074,
                         -0.6691306063588582,
                         -0.6819983600624984,
                         -0.6946583704589973,
                         -0.7071067811865475,
                         -0.7193398003386509,
                         -0.7313537016191701,
                         -0.743144825477394,
                         -0.7547095802227717,
                         -0.7660444431189779,
                         -0.7771459614569711,
                         -0.7880107536067221,
                         -0.7986355100472928,
                         -0.8090169943749473,
                         -0.8191520442889916,
                         -0.8290375725550414,
                         -0.8386705679454242,
                         -0.848048096156426,
                         -0.8571673007021121,
                         -0.8660254037844385,
                         -0.8746197071393959,
                         -0.882947592858927,
                         -0.8910065241883678,
                         -0.8987940462991668,
                         -0.9063077870366497,
                         -0.913545457642601,
                         -0.9205048534524403,
                         -0.9271838545667873,
                         -0.9335804264972016,
                         -0.9396926207859082,
                         -0.9455185755993168,
                         -0.9510565162951535,
                         -0.9563047559630353,
                         -0.961261695938319,
                         -0.9659258262890683,
                         -0.9702957262759965,
                         -0.9743700647852351,
                         -0.9781476007338056,
                         -0.9816271834476639,
                         -0.984807753012208,
                         -0.9876883405951377,
                         -0.9902680687415704,
                         -0.9925461516413221,
                         -0.9945218953682734,
                         -0.9961946980917455,
                         -0.9975640502598242,
                         -0.9986295347545738,
                         -0.9993908270190957,
                         -0.9998476951563913,
                         -1.0,
                         -0.9998476951563913,
                         -0.9993908270190958,
                         -0.9986295347545738,
                         -0.9975640502598243,
                         -0.9961946980917455,
                         -0.9945218953682734,
                         -0.992546151641322,
                         -0.9902680687415704,
                         -0.9876883405951378,
                         -0.9848077530122081,
                         -0.9816271834476641,
                         -0.9781476007338058,
                         -0.9743700647852352,
                         -0.9702957262759966,
                         -0.9659258262890682,
                         -0.9612616959383188,
                         -0.9563047559630354,
                         -0.9510565162951536,
                         -0.945518575599317,
                         -0.9396926207859085,
                         -0.9335804264972021,
                         -0.9271838545667874,
                         -0.9205048534524405,
                         -0.9135454576426008,
                         -0.9063077870366498,
                         -0.898794046299167,
                         -0.891006524188368,
                         -0.8829475928589271,
                         -0.8746197071393961,
                         -0.8660254037844386,
                         -0.8571673007021123,
                         -0.8480480961564261,
                         -0.8386705679454243,
                         -0.8290375725550421,
                         -0.8191520442889918,
                         -0.8090169943749476,
                         -0.798635510047293,
                         -0.7880107536067218,
                         -0.7771459614569708,
                         -0.7660444431189781,
                         -0.7547095802227722,
                         -0.7431448254773946,
                         -0.731353701619171,
                         -0.7193398003386517,
                         -0.7071067811865477,
                         -0.6946583704589976,
                         -0.6819983600624983,
                         -0.6691306063588581,
                         -0.6560590289905074,
                         -0.6427876096865396,
                         -0.6293203910498378,
                         -0.6156614753256588,
                         -0.6018150231520483,
                         -0.5877852522924734,
                         -0.5735764363510465,
                         -0.5591929034707473,
                         -0.544639035015027,
                         -0.5299192642332058,
                         -0.5150380749100545,
                         -0.5000000000000004,
                         -0.4848096202463369,
                         -0.4694715627858908,
                         -0.45399049973954697,
                         -0.438371146789077,
                         -0.4226182617407,
                         -0.40673664307580015,
                         -0.3907311284892747,
                         -0.37460659341591235,
                         -0.35836794954530077,
                         -0.3420201433256686,
                         -0.32556815445715753,
                         -0.3090169943749477,
                         -0.29237170472273627,
                         -0.2756373558169998,
                         -0.2588190451025207,
                         -0.24192189559966787,
                         -0.22495105434386534,
                         -0.20791169081775987,
                         -0.19080899537654467,
                         -0.17364817766693127,
                         -0.1564344650402311,
                         -0.13917310096006588,
                         -0.12186934340514811,
                         -0.10452846326765342,
                         -0.08715574274765832,
                         -0.06975647374412476,
                         -0.05233595624294437,
                         -0.034899496702500823,
                         -0.01745240643728445};

static QueueHandle_t i2sstateQueue    = nullptr;
static QueueHandle_t i2sAudioEndQueue = nullptr;

IRAM_ATTR double fastSin(double deg) {
    int integer = (int)deg % 360;
    // int point = deg - integer;
    return sinmap[integer];
}

void speakerPlayTask(void *arg) {
    i2sQueueMsg_t QueueMsg;
    while (1) {
        if (xQueueReceive(i2sstateQueue, &QueueMsg, portMAX_DELAY) == pdTRUE) {
            if (QueueMsg.type == kTypeAudio) {
                audioParameters_t *pam = (audioParameters_t *)QueueMsg.dataptr;
                size_t bytes_written   = 0;
                i2s_write(SPAKER_I2S_NUMBER, pam->pAudioData, pam->length,
                          &bytes_written, portMAX_DELAY);
                // Serial.printf("point :%p\r\n",pam->pAudioData);
                if (pam->freeFlag == true)
                    xQueueSend(i2sAudioEndQueue, &pam->pAudioData,
                               (TickType_t)0);
                // delay(1);
                // delete (uint16_t*)pam->pAudioData;
                delete pam;
            } else if (QueueMsg.type == kTypeBeep) {
                beepParameters_t *pam = (beepParameters_t *)QueueMsg.dataptr;
                size_t bytes_written  = 0;
                size_t count = 0, length = 0;

                double t = (1 / (double)pam->freq) * (double)pam->rate;

                if (pam->time > 1000) {
                    length = pam->rate * (pam->time % 1000) / 1000;
                    count  = pam->time / 1000;
                } else {
                    length = pam->rate * pam->time / 1000;
                    count  = 0;
                }
                int rawLength = (count == 0) ? length : pam->rate;
                rawLength -= (int)((int)(rawLength % (int)t));

                int16_t *raw = (int16_t *)calloc(rawLength, sizeof(int16_t));
                for (int i = 0; i < rawLength; i++) {
                    double val = 0;
                    if (i < 1000) {
                        val = pam->maxval * i / 1000;
                    } else if (i > (rawLength - 1000)) {
                        val = pam->maxval -
                              pam->maxval * (1000 - (rawLength - i)) / 1000;
                    } else {
                        val = pam->maxval;
                    }
                    if (pam->freq == 0) {
                        raw[i] = 0;
                    } else {
                        raw[i] = (int16_t)(((fastSin(360 * i / t) + 0) * val));
                    }
                }
                if (rawLength != 0) {
                    i2s_write(SPAKER_I2S_NUMBER, raw, (rawLength)*2,
                              &bytes_written, portMAX_DELAY);
                    Serial.printf("I2S Write\r\n");
                }
                delete pam;
                delete raw;
            }
        }
        delay(1);
    }
}

bool ATOMSPK::begin(int __rate) {
    esp_err_t err = ESP_OK;

    i2s_driver_uninstall(SPAKER_I2S_NUMBER);
    i2s_config_t i2s_config = {
        .mode        = (i2s_mode_t)(I2S_MODE_MASTER),
        .sample_rate = __rate,
        .bits_per_sample =
            I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
        .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 2,
        .dma_buf_len          = 128,
    };
    i2s_config.mode               = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_config.use_apll           = false;
    i2s_config.tx_desc_auto_clear = true;

    err += i2s_driver_install(SPAKER_I2S_NUMBER, &i2s_config, 0, NULL);
    i2s_pin_config_t tx_pin_config;

    tx_pin_config.bck_io_num   = CONFIG_I2S_BCK_PIN;
    tx_pin_config.ws_io_num    = CONFIG_I2S_LRCK_PIN;
    tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;
    tx_pin_config.data_in_num  = CONFIG_I2S_DATA_IN_PIN;

    err += i2s_set_pin(SPAKER_I2S_NUMBER, &tx_pin_config);

    err += i2s_set_clk(SPAKER_I2S_NUMBER, __rate, I2S_BITS_PER_SAMPLE_16BIT,
                       I2S_CHANNEL_MONO);

    _rate            = __rate;
    i2sAudioEndQueue = xQueueCreate(1024, sizeof(uint16_t *));
    i2sstateQueue    = xQueueCreate(1024, sizeof(i2sQueueMsg_t));
    if (i2sstateQueue == 0) {
        return false;
    }
    xTaskCreatePinnedToCore(speakerPlayTask, "speakerPlayTask", 4096, nullptr,
                            10, nullptr, 0);
    return (err == ESP_OK) ? true : false;
}

size_t ATOMSPK::playRAW(const uint8_t *__audioPtr, size_t __size, bool modal,
                        bool freeFlag, TickType_t __ticksToWait) {
    size_t writeSize = 0;
    if (modal == false) {
        audioParameters_t *pam =
            (audioParameters_t *)malloc(sizeof(audioParameters_t));
        pam->pAudioData   = (void *)__audioPtr;
        pam->length       = __size;
        pam->freeFlag     = freeFlag;
        i2sQueueMsg_t msg = {.type = kTypeAudio, .dataptr = pam};
        xQueueSend(i2sstateQueue, &msg, (TickType_t)0);
        // xTaskCreatePinnedToCore(speakerAudioTask, "speakerAudioTask", 4096,
        // pam, 3, NULL, 0);
    } else {
        i2s_write(SPAKER_I2S_NUMBER, __audioPtr, __size, &writeSize,
                  __ticksToWait);
    }
    return writeSize;
}

size_t ATOMSPK::playBeep(int __freq, int __timems, int __maxval, bool __modal) {
    size_t writeSize = 0;

    if (__modal == false) {
        beepParameters_t *pam =
            (beepParameters_t *)malloc(sizeof(beepParameters_t));
        pam->freq   = __freq;
        pam->time   = __timems;
        pam->rate   = _rate;
        pam->maxval = __maxval;

        i2sQueueMsg_t msg = {.type = kTypeBeep, .dataptr = pam};
        xQueueSend(i2sstateQueue, &msg, (TickType_t)0);
    } else {
        size_t bytes_written = 0;
        size_t count = 0, length = 0;

        double t = (1 / (double)__freq) * (double)_rate;

        if (__timems > 1000) {
            length = _rate * (__timems % 1000) / 1000;
            count  = __timems / 1000;
        } else {
            length = _rate * __timems / 1000;
            count  = 0;
        }
        int rawLength = (count == 0) ? length : _rate;

        uint16_t *raw = (uint16_t *)ps_calloc(rawLength, sizeof(uint16_t));

        for (int i = 0; i < rawLength; i++) {
            int val = 0;
            if (i < 100) {
                val = __maxval * i / 100;
            } else if (i > (rawLength - 1000)) {
                val = __maxval - __maxval * (1000 - (rawLength - i)) / 1000;
            } else {
                val = __maxval;
            }
            raw[i] = (uint16_t)((fastSin(360 / t * i)) * val);
        }

        for (int i = 0; i < count; i++) {
            i2s_write(SPAKER_I2S_NUMBER, raw, _rate, &bytes_written,
                      portMAX_DELAY);
        }
        if (length != 0) {
            i2s_write(SPAKER_I2S_NUMBER, raw, length, &bytes_written,
                      portMAX_DELAY);
        }
        delete raw;
    }
    return writeSize;
}