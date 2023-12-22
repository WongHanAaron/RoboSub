#include "ADC.hpp"

void Exodus::ADC::initialize(pin pinNumber, adc_base baseNumber){
    uint32_t inputChannel, peripheral;
    switch(pinNumber){
        case PE3:
            inputChannel = ADC_CTL_CH0;
            break;
        case PE2:
            inputChannel = ADC_CTL_CH1;
            break;
        case PE1:
            inputChannel = ADC_CTL_CH2;
            break;
        case PE0:
            inputChannel = ADC_CTL_CH3;
            break;
        case PD3:
            inputChannel = ADC_CTL_CH4;
            break;
        case PD2:
            inputChannel = ADC_CTL_CH5;
            break;
        case PD1:
            inputChannel = ADC_CTL_CH6;
            break;
        case PD0:
            inputChannel = ADC_CTL_CH7;
            break;
        case PE5:
            inputChannel = ADC_CTL_CH8;
            break;
        case PE4:
            inputChannel = ADC_CTL_CH9;
            break;
        case PB4:
            inputChannel = ADC_CTL_CH10;
            break;
        case PB5:
            inputChannel = ADC_CTL_CH11;
            break;
        default:
            return;
    }
    switch(baseNumber){
        case BASE_0:
            adcBase = ADC0_BASE;
            peripheral = SYSCTL_PERIPH_ADC0;
            break;
        case BASE_1:
            adcBase = ADC1_BASE;
            peripheral = SYSCTL_PERIPH_ADC1;
            break;
        default:
            return;
    }
    SysCtlPeripheralEnable(peripheral);

//    for (int i = 0; i < 10 ; i ++);   // Delay added to give time for the ADC to initialize (is my guess)
    SysCtlDelay(10); // Wait for pin to be ready
    ADCSequenceDisable(adcBase, 1);
    ADCSequenceConfigure(adcBase, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(adcBase, 1, 0, inputChannel);
    ADCSequenceStepConfigure(adcBase, 1, 1, inputChannel);
    ADCSequenceStepConfigure(adcBase, 1, 2, inputChannel);
    ADCSequenceStepConfigure(adcBase, 1, 3, inputChannel|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(adcBase, 1);
}

uint32_t Exodus::ADC::read(){
    volatile uint32_t averageReading;
    uint32_t ui32ADC0Value[4];

    ADCIntClear(this->adcBase, 1);
    ADCProcessorTrigger(this->adcBase, 1);

    while(!ADCIntStatus(this->adcBase, 1, false));

    ADCSequenceDataGet(this->adcBase, 1, ui32ADC0Value);
    averageReading = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
    return averageReading;
}
