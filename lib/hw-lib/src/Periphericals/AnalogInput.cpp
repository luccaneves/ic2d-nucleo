#include "hw/Periphericals/AnalogInput.h"


uint8_t AnalogInput::_using_ADC1 = 0;
uint8_t AnalogInput::_using_ADC2 = 0;
uint8_t AnalogInput::_using_ADC3 = 0;
uint32_t AnalogInput::counterNumberADC = 1;
uint16_t* AnalogInput::_pointer = nullptr;
uint16_t AnalogInput::last_element[NUMBER_ADC_CHANNELS_USED] = {0};
int AnalogInput::avg_acc[NUMBER_ADC_CHANNELS_USED] = {0};
int AnalogInput::avg_qnt[NUMBER_ADC_CHANNELS_USED] = {0};

bufferi_t AnalogInput::values_buffer_sensor[NUMBER_ADC_CHANNELS_USED];

// Only use this if you know what you are doing!!!
#define NO_ASSERT

// If you want a memory deallocation look for bufferi_free(bufferi_t *b).
// This is just a quick clear, without writing zeros, the data is still available.
void bufferi_clear(bufferi_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // initializing size as 0
    b->size = 0;

    // initializing cursor as 0
    b->cur = 0;
}

void bufferi_init(bufferi_t *b, size_t max_size)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // allocate the requested size
    b->data = (uint16_t *)malloc(max_size * sizeof(uint16_t));

    // check if data allocation was successful
    if (b->data != NULL)
    {
        // setting up a valid max_size after checking allocation
        b->max_size = max_size;
    }
    else
    {
        // setting up a valid max_size after failing allocation
        b->max_size = 0;
    }

    // initializing size and cur as 0
    bufferi_clear(b);
}

void bufferi_free(bufferi_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if data is allocated before trying to deallocate
    assert(b->data != NULL);
#endif

    // free data
    free(b->data);

    // just making sure the previous pointer is invalid
    b->data = NULL;

    // making sure the max_size verifications will be coherent
    b->max_size = 0;

    // making sure the size and cur verifications will be coherent
    bufferi_clear(b);
}

uint16_t *bufferi_at(bufferi_t *b, size_t pos)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if buffer has data
    assert(b->data != NULL);
#endif

    size_t circular_pos = (pos + b->cur) % b->max_size;

    // check if position is valid in buffer data
    // assert(circular_pos > -1 && circular_pos < b->max_size);

    // return pointer to circular position in buffer data
    return &(b->data[circular_pos]);
}

uint16_t bufferi_get(bufferi_t *b, size_t pos)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if buffer has data
    assert(b->data != NULL);

    // check if position is valid
    assert((pos >= 0) && (pos < b->size));
#endif

    // return value from circular position in buffer data
    return *bufferi_at(b, pos);
}

void bufferi_push_back(bufferi_t *b, uint16_t value)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if circular buffer is not full
    assert(b->size < b->max_size);
#endif

    // get possible position for new value
    uint16_t *data = bufferi_at(b, b->size);

    // set data with value
    *data = value;

    // increment the circular buffer size
    b->size++;
}

void bufferi_pop_front(bufferi_t *b, uint16_t *value)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if circular buffer is not empty
    assert(b->size > 0);
#endif

    // get possible position for first value
    uint16_t *data = bufferi_at(b, 0);

    // sets value from first element in the circular buffer
    if (value != NULL)
    {
        *value = *data;
    }

    // decrement the circular buffer size
    b->size--;

    // move buffer cursor
    b->cur = (b->cur + 1) % b->max_size;
}

void bufferi_print(bufferi_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    printf("bufferi:");
    for (size_t p = 0; p < b->size; ++p)
    {
        printf(" %d", bufferi_get(b, p));
    }
    printf("\n");
}

void bufferi_push_and_pop(bufferi_t *b, uint16_t push_value, uint16_t *pop_value)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // pop front before overflowing
    bufferi_pop_front(b, pop_value);

    // push back after removing first element
    bufferi_push_back(b, push_value);
}

void bufferi_avgi(bufferi_t *b, uint16_t *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    int acc = 0;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += bufferi_get(b, p);
    }
    *avg = acc / b->size;
}

void bufferi_avgd(bufferi_t *b, double *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    double acc = 0.0;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += (double)bufferi_get(b, p);
    }
    *avg = acc / ((double)b->size);
}

void bufferi_avgf(bufferi_t *b, float *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    float acc = 0.0f;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += (float)bufferi_get(b, p);
    }
    *avg = acc / ((float)b->size);
}


void interrupt_dma(){
    //clear flag
    int i = 0;
    int j =0;
    for(i = 0; i < NUMBER_ADC_CHANNELS_USED; i++){
        for(j = 0; j < TESTE; j++)
        {
            if (AnalogInput::values_buffer_sensor[i].size < AnalogInput::values_buffer_sensor[i].max_size)
            {
                bufferi_push_back(AnalogInput::values_buffer_sensor + i, AnalogInput::_pointer[TESTE*j + i]); // O(1)
            }
            else
            {
                bufferi_push_and_pop(AnalogInput::values_buffer_sensor + i, AnalogInput::_pointer[TESTE*j + i], AnalogInput::last_element + i); // O(1)
                AnalogInput::avg_acc[i] -= AnalogInput::last_element[i];                                  // O(1)
                    // printf("subtract %d == %d\n", last_element, avg_acc);
            }
            AnalogInput::avg_acc[i] += AnalogInput::_pointer[TESTE*j + i];

            //AnalogInput::avg_acc[i] = AnalogInput::_pointer[i];
            
            AnalogInput::avg_qnt[i] = AnalogInput::values_buffer_sensor[i].size;
        }
    }

    DMA2->LIFCR |= (1 << 5);//CLEAR INTERRUPT FLAG FOR CHANNEL 0 OF DMA2


}

/*void DMA2_Stream0_IRQHandler(){

}*/

AnalogInput::AnalogInput(PinName pin) {
    bool first_instance;

    //default setting for ADC
    #ifdef TARGET_STM32L4
        ADCPrescaler Prescaler = ADC_SPCLK1;
        ADCAlign Alignment = ADC_Right;
        ADCSample Sample = ADC_2s5;
        ADCResolution Resolution = ADC_12b;
    #endif

    #ifdef TARGET_STM32F4
        ADCPrescaler Prescaler = ADC_PCLK2;
        ADCAlign Alignment = ADC_Right;
        ADCSample Sample = ADC_3s;
        ADCResolution Resolution = ADC_12b;
    #endif

    ADC_Common_TypeDef ADC_Common;
    uint32_t function = pinmap_function(pin, PinMap_ADC);
    //find for ADC (1, 2 or 3)
    _Conversor = (ADC_TypeDef *)pinmap_peripheral(pin, PinMap_ADC);
    //configure the GPIO
    pinmap_pinout(pin, PinMap_ADC);
    //find for channel (1, 2, ... or 15)
    _Channel = STM_PIN_CHANNEL(function);
    //enable the clock of the ADC
    #ifdef TARGET_STM32L4
        #if defined(ADC1)
            if ((ADCName &)_Conversor == ADC_1) {
                __HAL_RCC_ADC_CLK_ENABLE();
                if (_using_ADC1 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC1++;
            }
        #endif
        #if defined(ADC2)
            if ((ADCName &)_Conversor == ADC_2) {
                __HAL_RCC_ADC_CLK_ENABLE();
                if (_using_ADC2 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC2++;
            }
        #endif

        if (first_instance)
        {
            if ((Prescaler == ADC_SPCLK1) || (Prescaler == ADC_SPCLK2) || (Prescaler == ADC_SPCLK4)) {
                RCC->CCIPR |= RCC_ADCCLKSOURCE_SYSCLK;
                //HAL_RCC_GetSysClockFreq();
            }
            else {
                RCC->CCIPR |= RCC_ADCCLKSOURCE_PLLSAI1;
                RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;
                RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN;
                __HAL_RCC_PLLSAI1_ENABLE();
            }
        }

    #elif TARGET_STM32F4
        #if defined(ADC1)
            if ((ADCName &)_Conversor == ADC_1) {
                __HAL_RCC_ADC1_CLK_ENABLE();
                if (_using_ADC1 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC1++;
            }
        #endif
        #if defined(ADC2)
            if ((ADCName &)_Conversor == ADC_2) {
                __HAL_RCC_ADC2_CLK_ENABLE();
                if (_using_ADC2 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC2++;
            }
        #endif
        #if defined(ADC3)
            if ((ADCName &)_Conversor == ADC_3) {
                __HAL_RCC_ADC3_CLK_ENABLE();
                if (_using_ADC3 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC3++;
            }
        #endif
    #endif
    //configure the clock prescaler of the ADC
    if (first_instance) ADC_Common.CCR = Prescaler;

    //configure the ADC
    #ifdef TARGET_STM32L4
        if (first_instance) _Conversor->CFGR = Resolution | Alignment;
        _Conversor->CFGR2 = 0U;
    #elif TARGET_STM32F4
        if (first_instance)
        {
            _Conversor->CR1 = Resolution;
            _Conversor->CR2 = Alignment;
        }
    #endif
    
    _continuous_mode = false;
    _usage_dma = false;
    _usage_fast_divider = false;
    _injection_convertion = true;

    //set initializated flag
    _initialized = true;
}

AnalogInput::AnalogInput(PinName pin, ADCPrescaler Prescaler, ADCAlign Alignment, ADCSample Sample, ADCResolution Resolution) {
    bool first_instance;
    ADC_Common_TypeDef ADC_Common;
    uint32_t function = pinmap_function(pin, PinMap_ADC);
    //find for ADC (1, 2 or 3)
    _Conversor = (ADC_TypeDef *)pinmap_peripheral(pin, PinMap_ADC);
    //configure the GPIO
    pinmap_pinout(pin, PinMap_ADC);
    //find for channel (1, 2, ... or 15)
    _Channel = STM_PIN_CHANNEL(function);
    //enable the clock of the ADC
    #ifdef TARGET_STM32L4
        #if defined(ADC1)
            if ((ADCName &)_Conversor == ADC_1) {
                __HAL_RCC_ADC_CLK_ENABLE();
                if (_using_ADC1 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC1++;
            }
        #endif
        #if defined(ADC2)
            if ((ADCName &)_Conversor == ADC_2) {
                __HAL_RCC_ADC_CLK_ENABLE();
                if (_using_ADC2 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC2++;
            }
        #endif

        if (first_instance)
        {
            if ((Prescaler == ADC_SPCLK1) || (Prescaler == ADC_SPCLK2) || (Prescaler == ADC_SPCLK4)) {
                RCC->CCIPR |= RCC_ADCCLKSOURCE_SYSCLK;
                //HAL_RCC_GetSysClockFreq();
            }
            else {
                RCC->CCIPR |= RCC_ADCCLKSOURCE_PLLSAI1;
                RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;
                RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN;
                __HAL_RCC_PLLSAI1_ENABLE();
            }
        }

    #elif TARGET_STM32F4
        #if defined(ADC1)
            if ((ADCName &)_Conversor == ADC_1) {
                __HAL_RCC_ADC1_CLK_ENABLE();
                if (_using_ADC1 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC1++;
            }
        #endif
        #if defined(ADC2)
            if ((ADCName &)_Conversor == ADC_2) {
                __HAL_RCC_ADC2_CLK_ENABLE();
                if (_using_ADC2 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC2++;
            }
        #endif
        #if defined(ADC3)
            if ((ADCName &)_Conversor == ADC_3) {
                __HAL_RCC_ADC3_CLK_ENABLE();
                if (_using_ADC3 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC3++;
            }
        #endif
    #endif
    //configure the clock prescaler of the ADC
    if (first_instance) ADC_Common.CCR = Prescaler;
    
    //configure the ADC
    #ifdef TARGET_STM32L4
        if (first_instance) _Conversor->CFGR = Resolution | Alignment;
        _Conversor->CFGR2 = 0U;
    #elif TARGET_STM32F4
        if (first_instance)
        {
            _Conversor->CR1 = Resolution;
            _Conversor->CR2 = Alignment;
        }
    #endif
    
    _continuous_mode = false;
    _usage_dma = false;
    _usage_fast_divider = false;
    _injection_convertion = true;

    //set initializated flag
    _initialized = true;
}

AnalogInput::AnalogInput(uint32_t channel, ADCPrescaler Prescaler, ADCAlign Alignment, ADCSample Sample, ADCResolution Resolution, ADCContinuous Continuous, ADCDma Dma, uint32_t Size_buffer) {
    bool first_instance;
    uint32_t i, j, k;
    ADC_Common_TypeDef ADC_Common;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*uint32_t function = pinmap_function(pin, PinMap_ADC);
    //find for ADC (1, 2 or 3)
    _Conversor = (ADC_TypeDef *)pinmap_peripheral(pin, PinMap_ADC);
    //configure the GPIO
    pinmap_pinout(pin, PinMap_ADC);*/
    //find for channel (1, 2, ... or 15)

    _Channel = channel;
    _Conversor = ADC1;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    
    */
    
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //enable the clock of the ADC


    //bufferi_init(&values_buffer, NUMBER_ADC_CHANNELS_USED);


    conversionRank = counterNumberADC;

    #ifdef TARGET_STM32L4
        #if defined(ADC1)
            if ((ADCName &)_Conversor == ADC_1) {
                __HAL_RCC_ADC_CLK_ENABLE();
                if (_using_ADC1 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC1++;
            }
        #endif
        #if defined(ADC2)
            if ((ADCName &)_Conversor == ADC_2) {
                __HAL_RCC_ADC_CLK_ENABLE();
                if (_using_ADC2 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC2++;
            }
        #endif

        if (first_instance)
        {
            if ((Prescaler == ADC_SPCLK1) || (Prescaler == ADC_SPCLK2) || (Prescaler == ADC_SPCLK4)) {
                RCC->CCIPR |= RCC_ADCCLKSOURCE_SYSCLK;
                //HAL_RCC_GetSysClockFreq();
            }
            else {
                RCC->CCIPR |= RCC_ADCCLKSOURCE_PLLSAI1;
                RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;
                RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN;
                __HAL_RCC_PLLSAI1_ENABLE();
            }
        }

    #elif TARGET_STM32F4
        #if defined(ADC1)
            if ((ADCName &)_Conversor == ADC_1) {
                
                if (_using_ADC1 == 0) {
                    __HAL_RCC_ADC1_CLK_ENABLE();
                    first_instance = true;
                }
                
                else first_instance = false;
                _using_ADC1++;
            }
        #endif
        #if defined(ADC2)
            if ((ADCName &)_Conversor == ADC_2) {
                __HAL_RCC_ADC2_CLK_ENABLE();
                if (_using_ADC2 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC2++;
            }
        #endif
        #if defined(ADC3)
            if ((ADCName &)_Conversor == ADC_3) {
                __HAL_RCC_ADC3_CLK_ENABLE();
                if (_using_ADC3 == 0) first_instance = true;
                else first_instance = false;
                _using_ADC3++;
            }
        #endif
    #endif
    //configure the clock prescaler of the ADC
    if (first_instance) ADC_Common.CCR = Prescaler;

    #ifdef TARGET_STM32L4
        //configure the time sample
        if (_Channel < 10)
        {
            _Conversor->SMPR1 = Sample<<(_Channel*3);
            //_Conversor->SMPR2 = 0U;
        }
        else
        {
            //_Conversor->SMPR1 = 0U;
            _Conversor->SMPR2 = Sample<<(_Channel*3);
        }
        //configure the channel for convertion
        _Conversor->SQR1 = _Channel<<6;
        _Conversor->SQR2 = 0U;
        _Conversor->SQR3 = 0U;
        _Conversor->SQR4 = 0U;
    #elif TARGET_STM32F4
        //configure the time sample
      /*  if (_Channel < ADC_CHANNEL_10)
        {
            //_Conversor->SMPR1 = 0U;
            //_Conversor->SMPR2 = Sample<<(_Channel*3);


            _Conversor->SMPR2 &= ~ADC_SMPR2(ADC_SMPR2_SMP0, _Channel);
                
    
            _Conversor->SMPR2 |= ADC_SMPR2(Sample, _Channel);

            
        }
        else
        {
            _Conversor->SMPR1 &= ~ADC_SMPR1(ADC_SMPR1_SMP10, _Channel);

            _Conversor->SMPR1 |= ADC_SMPR1(Sample, _Channel);

            //_Conversor->SMPR1 = Sample<<(_Channel*3);
            //_Conversor->SMPR2 = 0U;
        }*/
        //configure the channel for convertion


       /*if (conversionRank < 7U)
        {

            _Conversor->SQR3 &= ~ADC_SQR3_RK(ADC_SQR3_SQ1, conversionRank);
            
 
            _Conversor->SQR3 |= ADC_SQR3_RK(_Channel, conversionRank);
        }
     
        else if (conversionRank < 13U)
        {

            _Conversor->SQR2 &= ~ADC_SQR2_RK(ADC_SQR2_SQ7, conversionRank);
            
       
            _Conversor->SQR2 |= ADC_SQR2_RK(_Channel, conversionRank);
        }
   
        else
        {
    
            _Conversor->SQR1 &= ~ADC_SQR1_RK(ADC_SQR1_SQ13, conversionRank);
            
     
            _Conversor->SQR1 |= ADC_SQR1_RK(_Channel, conversionRank);
        }*/

        //for (int i = 0; i < 9; i++){
            //_Conversor->SMPR1 |= (7U << 3*i);
            //_Conversor->SMPR2 |= (7U << 3*i);
        //}

        _Conversor->SMPR1 |= (7U << 3*5);//Setar sampling time do ADC do canal 15
        _Conversor->SMPR1 |= (7U << 3*4);//Setar sampling time do ADC do canal 14

        _Conversor->SMPR1 |= (7U << 3*3);//Setar sampling time do ADC do canal 13
        _Conversor->SMPR1 |= (7U << 3*2);//Setar sampling time do ADC do canal 12
        _Conversor->SMPR1 |= (7U << 3*1);//Setar sampling time do ADC do canal 11
        _Conversor->SMPR1 |= (7U);//Setar sampling time do ADC do canal 10

        _Conversor->SMPR2 |= (7U << 3*4);////Setar sampling time do ADC do canal 4
        _Conversor->SMPR2 |= (7U << 3*1);////Setar sampling time do ADC do canal 1

        _Conversor->SQR1 |= ((NUMBER_ADC_CHANNELS_USED - 1) << 20);
        _Conversor->SQR2 = /*(14) + */(15 << 0);
        _Conversor->SQR3 = (4 << 0) + (1 << 5) + (10 << 10) + (14 << 25) + (12 << 15) + (13 << 20);

    #endif

    counterNumberADC++;
    
    //check if using DMA
    if (Size_buffer > 65535) _Size_buffer = 65535;
    else if (Size_buffer == 0) _Size_buffer = 1;
    else _Size_buffer = Size_buffer;

    if (Dma == ADC_Dma)
    {
        if(counterNumberADC == (NUMBER_ADC_CHANNELS_USED + 1)){
            _pointer = (uint16_t *)malloc(_Size_buffer*2);
            #ifdef TARGET_STM32L4
                #if defined(ADC1)
                    if ((ADCName &)_Conversor == ADC_1)
                    {
                        if ((DMA1_Channel1->CCR & DMA_CCR_EN) == 0)
                        {
                            if(__DMA1_IS_CLK_DISABLED()) __DMA1_CLK_ENABLE();
                            _Stream_DMA = DMA1_Channel1;
                            DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & 0xFFFFFFF0) | (DMA_REQUEST_0<<0);
                        }
                        else if ((DMA2_Channel3->CCR & DMA_CCR_EN) == 0)
                        {
                            if(__DMA2_IS_CLK_DISABLED()) __DMA2_CLK_ENABLE();
                            _Stream_DMA = DMA2_Channel3;
                            DMA2_CSELR->CSELR = (DMA2_CSELR->CSELR & 0xFFFFF0FF) | (DMA_REQUEST_0<<8);
                        }
                    }
                #endif
                #if defined(ADC2)
                    if ((ADCName &)_Conversor == ADC_2)
                    {
                        if ((DMA1_Channel2->CCR & DMA_CCR_EN) == 0)
                        {
                            if(__DMA1_IS_CLK_DISABLED()) __DMA1_CLK_ENABLE();
                            _Stream_DMA = DMA1_Channel2;
                            DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & 0xFFFFFF0F) | (DMA_REQUEST_0<<4);
                        }
                        else if ((DMA2_Channel4->CCR & DMA_CCR_EN) == 0)
                        {
                            if(__DMA2_IS_CLK_DISABLED()) __DMA2_CLK_ENABLE();
                            _Stream_DMA = DMA2_Channel4;
                            DMA2_CSELR->CSELR = (DMA2_CSELR->CSELR & 0xFFFF0FFF) | (DMA_REQUEST_0<<12);
                        }
                    }
                #endif
                _Stream_DMA->CCR = DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PL;
                _Stream_DMA->CNDTR = _Size_buffer;
                _Stream_DMA->CPAR = (uint32_t)&_Conversor->DR;
                _Stream_DMA->CMAR = (uint32_t)_pointer;
                _Stream_DMA->CCR |= DMA_CCR_EN;
            #elif TARGET_STM32F4
                if (__DMA2_IS_CLK_DISABLED()) __DMA2_CLK_ENABLE();
                #if defined(ADC1)
                    if ((ADCName &)_Conversor == ADC_1)
                    {
                        if ((DMA2_Stream0->CR & 0x1) == 0)
                        {
                            _Stream_DMA = DMA2_Stream0;
                            _Stream_DMA->CR = 0U;
                        }
                        else if ((DMA2_Stream4->CR & 0x1) == 0)
                        {
                            _Stream_DMA = DMA2_Stream4;
                            _Stream_DMA->CR = 0U;
                        }
                    }
                #endif
                #if defined(ADC2)
                    if ((ADCName &)_Conversor == ADC_2) {
                        if ((DMA2_Stream2->CR & 0x1) == 0)
                        {
                            _Stream_DMA = DMA2_Stream2;
                            _Stream_DMA->CR = 0x02000000;
                        }
                        else if ((DMA2_Stream3->CR & 0x1) == 0)
                        {
                            _Stream_DMA = DMA2_Stream3;
                            _Stream_DMA->CR = 0x02000000;
                        }
                    }
                #endif
                #if defined(ADC3)
                    if ((ADCName &)_Conversor == ADC_3) {
                        if ((DMA2_Stream0->CR & 0x1) == 0)
                        {
                            _Stream_DMA = DMA2_Stream0;
                            _Stream_DMA->CR = 0x04000000;
                        }
                        else if ((DMA2_Stream1->CR & 0x1) == 0)
                        {
                            _Stream_DMA = DMA2_Stream1;
                            _Stream_DMA->CR = 0x04000000;
                        }
                    }
                #endif
                _Stream_DMA->CR |= 0x00032D00;
                _Stream_DMA->NDTR = _Size_buffer;
                _Stream_DMA->PAR = (uint32_t)&_Conversor->DR;
                _Stream_DMA->M0AR = (uint32_t)_pointer;
                _Stream_DMA->CR |= (1 << 4); //Habilitar interrupt transfer complete
                _Stream_DMA->CR |= 1;

                HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
                NVIC_SetVector(DMA2_Stream0_IRQn, (uint32_t)interrupt_dma);
                NVIC_EnableIRQ(DMA2_Stream0_IRQn);

                bufferi_init(values_buffer_sensor + 0, NUMBER_READS_PER_CHANNELS);
                bufferi_init(values_buffer_sensor + 1, NUMBER_READS_PER_CHANNELS);
                bufferi_init(values_buffer_sensor + 2, NUMBER_READS_PER_CHANNELS);
                bufferi_init(values_buffer_sensor + 3, NUMBER_READS_PER_CHANNELS);
                bufferi_init(values_buffer_sensor + 4, NUMBER_READS_PER_CHANNELS);
                bufferi_init(values_buffer_sensor + 5, NUMBER_READS_PER_CHANNELS);
                bufferi_init(values_buffer_sensor + 6, NUMBER_READS_PER_CHANNELS);
            #endif
            Continuous = ADC_Continuous;
        }
        else{
            //_pointer = (uint16_t *)_Stream_DMA->M0AR;
        }
    }
    //configure the ADC
    #ifdef TARGET_STM32L4
        if (first_instance) _Conversor->CFGR = Dma | Resolution | Alignment | Continuous;
        else _Conversor->CFGR |= (Dma | Continuous);
        _Conversor->CFGR2 = 0U;
    #elif TARGET_STM32F4
        if (first_instance)
        {
            _Conversor->CR1 = Resolution | (1 << 8);
            //_Conversor->CR1 |= (1 << 5) + (1 << 26); //Habilitar interrupt do ADC. Talvez desnecessário...
            _Conversor->CR2 = Alignment | Continuous | Dma;
        }
        else _Conversor->CR2 |= (Continuous | Dma);
    #endif
    //check configuration
    if (Continuous == ADC_Continuous) _continuous_mode = true;
    else _continuous_mode = false;
    if (Dma == ADC_Dma) _usage_dma = true;
    else _usage_dma = false;
    
    //determine the divider coeficiente
    k = _Size_buffer;
    j = 0;
    for (i = 0; i < 32; i++)
    {
        if ((k & 0x1) != 0) j++;
        k>>1;
    }
    if (j > 1)
    {
        _usage_fast_divider = false;
        _Divider = _Size_buffer;
    }
    else 
    {
        _usage_fast_divider = true;
        k = _Size_buffer;
        _Divider = 0;
        do
        {
            k = k>>1;
            _Divider++;
        } while ((k & 0x1) == 0);
    }

    _injection_convertion = false;

    //set initializated flag
    _initialized = true;
}

AnalogInput::~AnalogInput() {
    #ifdef TARGET_STM32L4
        #if defined(ADC1)
            if ((ADCName &)_Conversor == ADC_1) {
                _using_ADC1--;
            }1
        #endif
        #if defined(ADC2)
            if ((ADCName &)_Conversor == ADC_2) {
                _using_ADC2--;
            }
        #endif
    #elif TARGET_STM32F4
        #if defined(ADC1)
            if ((ADCName &)_Conversor == ADC_1) {
                _using_ADC1--;
            }
        #endif
        #if defined(ADC2)
            if ((ADCName &)_Conversor == ADC_2) {
                _using_ADC2--;
            }
        #endif
        #if defined(ADC3)
            if ((ADCName &)_Conversor == ADC_3) {
                _using_ADC3--;
            }
        #endif
    #endif
}

int8_t AnalogInput::enable()
{
    u_int32_t timeout = 0;
    if (_initialized) //Check if the ADC was initialized
    {
        #ifdef TARGET_STM32L4
            if (is_enabled() == 0) //Check if the ADC is enable
            {
                _Conversor->CR &= ~ADC_CR_DEEPPWD; //Ensure the ADC out deep power
                _Conversor->CR |= ADC_CR_ADVREGEN; //Enable the ADC regulator
                wait_us(25); //Wait for regulater power on
                _Conversor->CR |= ADC_CR_ADCAL; //Start calibration sequence
                timeout = 5000000;
                while ((_Conversor->CR & ADC_CR_ADCAL) != 0U) //Wait for calibration
                {
                    timeout--;
                    if (timeout == 0) return(-2);
                }
                wait_us(1); //Ensure time after calibration
                _Conversor->ISR |= ADC_ISR_ADRDY; //Reset bit ADC enable
                _Conversor->CR |= ADC_CR_ADEN; //Start enable sequence
                timeout = 5000000;
                while ((_Conversor->ISR & ADC_ISR_ADRDY) != 0U) //Wait for enable is complited
                {
                    timeout--;
                    if (timeout == 0) return(-3);
                }
            }
            else
            {
                if (_continuous_mode && is_started() == 0)
                {
                    start();
                    return(2);
                }
                return(1); //ADC already is enable
            }
        #elif TARGET_STM32F4
            _Conversor->CR2 |= 0x00000001; //Enable the ADC
        #endif
        if (_continuous_mode) //Check if the continuous mode is active
            start(); //Start the convertion
        return(0); //Enabled with success
    }
    else return(-1); //ADC not initialized
}

int8_t AnalogInput::unable()
{
    if (_initialized) //Check if the ADC was initialized
    {
        stop(); //Stop the convertion
        #ifdef TARGET_STM32L4
            if (is_enabled() == 1) //Check if the ADC is unable
            {
                _Conversor->CR |= ADC_CR_ADDIS; //Start unable sequence
                while (is_enabled() == 1); //Wait for unable is complited
            }
            else return(1); //ADC already is unable
        #elif TARGET_STM32F4
            _Conversor->CR2 &= 0xFFFFFFFE; //Unable the ADC
        #endif
        return(0); //Unabled with success
    }
    else return(-1); //ADC not initialized

}

int8_t AnalogInput::start()
{
    if (_initialized) //Check if the ADC was initialized
    {
        #ifdef TARGET_STM32L4
            if (is_enabled() == 1) //Check if the ADC is enable
            {
                if (is_started() == 0) //Check if the conversion is in progress
                {
                    if (_injection_convertion)
                    {
                        _Conversor->CFGR |= (ADC_CFGR_JQDIS | ADC_CFGR_JDISCEN);
                        _Conversor->JSQR = _Channel<<8;
                        _Conversor->CR |= ADC_CR_JADSTART;
                    }
                    else
                    {
                        _Conversor->CR |= ADC_CR_ADSTART; //Start conversion
                    }
                }
                else
                    return(1); //Conversion already is in progress
            }
            else
                return(-2); //ADC not enabled
        #elif TARGET_STM32F4
            if (_injection_convertion)
            {
                _Conversor->CR1 &= ~ADC_CR1_JAUTO;
                _Conversor->CR1 |= ADC_CR1_JDISCEN;
                _Conversor->JSQR = _Channel<<15;
                _Conversor->CR2 |= ADC_CR2_JSWSTART;
            }
            else
            {
                _Conversor->CR2 |= ADC_CR2_SWSTART; //Start convertion
            }
        #endif
        return(0); //Conversion started with success
    }
    else return(-1); //ADC not initialized
}

int8_t AnalogInput::stop()
{
    if (_initialized) //Check if the ADC was initialized
    {
        #ifdef TARGET_STM32L4
            if (is_started() == 1) //Check if have the conversion is in progress
            {
                if (_injection_convertion)
                {
                    _Conversor->CR |= ADC_CR_JADSTP;
                    while((_Conversor->CR & ADC_CR_JADSTP) != 0U);
                }
                else
                {
                    _Conversor->CR |= ADC_CR_ADSTP; //Start stop conversion sequence
                    while((_Conversor->CR & ADC_CR_ADSTP) != 0U); //Wait finish stop sequence
                }
            }
        #elif TARGET_STM32F4
            _Conversor->CR2 &= 0xBFFFFFFF; //Stop convertion
        #endif
        return(0); //Convertion stoped with success
    }
    else return(-1); //ADC not initialized
}

uint8_t AnalogInput::is_enabled()
{
    #ifdef TARGET_STM32L4
        if ((_Conversor->CR & ADC_CR_ADEN) != 0U) //Check if the ADC is enable
            return 1; //ADC is enable
    #elif TARGET_STM32F4
        if ((_Conversor->CR2 & 0x00000001) != 0U) //Check if the ADC is enable
            return 1; //ADC is enable
    #endif
    else
        return 0; //ADC is unable
}

uint8_t AnalogInput::is_started()
{
    #ifdef TARGET_STM32L4
        if (_injection_convertion)
        {
            if ((_Conversor->CR & ADC_CR_JADSTART) != 0U) //Check if the conversion is in progress
                return 1; //Convertion is in progress
        }
        else
        {
            if ((_Conversor->CR & ADC_CR_ADSTART) != 0U) //Check if the conversion is in progress
                return 1; //Convertion is in progress
        }
    #elif TARGET_STM32F4
        if ((_Conversor->SR & 0x00000010) != 0) //Check if the conversion is in progress
            return 1; //Convertion is in progress
    #endif
    return 0; //Convertion isn't in progress
}

uint8_t AnalogInput::data_ready()
{
    #ifdef TARGET_STM32L4
        if (_injection_convertion)
        {
            if ((_Conversor->ISR & ADC_ISR_JEOC) != 0U)
                return 1;
        }
        else
        {
            if ((_Conversor->ISR & ADC_ISR_EOC) != 0U) //Check if the conversion completed
                return 1; //Conversion completed
        }
    #elif TARGET_STM32F4
        if (_injection_convertion)
        {
            if ((_Conversor->SR & ADC_SR_JEOC) != 0)
                return 1;
        }
        else
        {
            if ((_Conversor->SR & ADC_SR_EOC) != 0) //Check if the conversion completed
                return 1; //Conversion completed
        }
    #endif
        return 0; //Conversion didn't complete
}

uint16_t AnalogInput::read_converted_value()
{
    if (_injection_convertion) return _Conversor->JDR1;
    else return _Conversor->DR; //Read the converted value
}

void AnalogInput::reset_complete_flag()
{
    #ifdef TARGET_STM32L4
        if (_injection_convertion) _Conversor->ISR |= ADC_ISR_JEOC;
        else _Conversor->ISR |= ADC_ISR_EOC; //Reset the complete conversion flag
    #elif TARGET_STM32F4
        if (_injection_convertion) _Conversor->SR &= ~ADC_SR_JEOC;
        else _Conversor->SR &= ~ADC_SR_EOC; //Reset the complete conversion flag
    #endif
}

float AnalogInput::read_average_float()
{
    uint16_t average = 0;
    if (_injection_convertion) average = read_last_word();
    else average = read_average_word();
    #ifdef TARGET_STM32L4
        if ((_Conversor->CFGR & ADC_CFGR_ALIGN) != 0) return ((float)average)/((float)0xFFFF);
    #elif TARGET_STM32F4
        if ((_Conversor->CR2 & ADC_CR2_ALIGN) != 0) return ((float)average)/((float)0xFFFF);
    #endif
    else return ((float)average)/((float)0x0FFF);
}

uint32_t AnalogInput::read_average_word()
{
    uint32_t i;
    uint32_t sum = 0;
    uint32_t average = 0;

    if (is_enabled() == 0) return 0;

    if (_injection_convertion)
    {
        average = read_last_word();
    }
    else if (_usage_dma)
    {

        average = (avg_acc[conversionRank - 1] >> NUMBER_BIT_SHIFT);
        


        /*for (i = 0; i < 10; i++) 
            sum += _pointer[NUMBER_ADC_CHANNELS_USED*(i) + (conversionRank - 1)];

        average = sum/10;*/

    }
    else if (_continuous_mode)
    {
        average = read_converted_value();
        reset_complete_flag();
    }
    else
    {
        for (i = 0; i < _Size_buffer; i++) sum += read_last_word();
        if (_usage_fast_divider) average = sum>>_Divider;
        else average = sum/_Divider;
    }
    return(average);
}

float AnalogInput::read_last_float()
{
    uint16_t last_value = read_last_word();
    #ifdef TARGET_STM32L4
        if ((_Conversor->CFGR & ADC_CFGR_ALIGN) != 0) return ((float)last_value)/((float)0xFFFF);
    #elif TARGET_STM32F4
        if ((_Conversor->CR2 & ADC_CR2_ALIGN) != 0) return ((float)last_value)/((float)0xFFFF);
    #endif
    else return ((float)last_value)/((float)0x0FFF);
}

uint16_t AnalogInput::read_last_word()
{
    uint16_t last_value;
    uint32_t last_pointer;
    uint32_t actual_buffer_position;

    if (is_enabled() == 0) return 0;

    if (_usage_dma)
    {
        #ifdef TARGET_STM32L4
        actual_buffer_position = _Stream_DMA->CNDTR;
        #elif TARGET_STM32F4
            actual_buffer_position = _Stream_DMA->NDTR;
        #endif
        last_pointer = _Size_buffer - actual_buffer_position;
        if (last_pointer == 0) last_pointer = _Size_buffer-1;
        else last_pointer -= 1;
        last_value = _pointer[(NUMBER_ADC_CHANNELS_USED)*(NUMBER_READS_PER_CHANNELS - 1) + (conversionRank - 1)];
    
    }
    else if (_continuous_mode)
    {
        last_value = read_converted_value();
        reset_complete_flag();
    }
    else
    {
        start();
        while (data_ready() == 0);
        last_value = read_converted_value();
        reset_complete_flag();
    }
    return last_value;
}
