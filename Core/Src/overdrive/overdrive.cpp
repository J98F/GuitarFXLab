#include "daisy_petal.h"
#include "daisysp.h"
#include "terrarium.h"

using namespace terrarium;
using namespace daisy;
using namespace daisysp;

#define SAMPLE_TIME 0.00002083333 // 1/fs, fs = 48kHz
#define PI 3.14159265359
#define PI_2 6.28318530718
#define LOWPASS 1
#define HIGHPASS 2
#define FILTER_TAP_NUM 53

typedef struct signal_buffer {
    float in;
    float out;
    float clipped;
    float lowpass_filtered;
    float highpass_filtered;
} signal_buffer;

typedef struct pedal_attributes {
    /* Footswitch */
    bool FS1_bypass = true;
    bool FS2_bypass = true;
    /* Switches */
    bool switch_1 = false;
    bool switch_2 = false;
    bool switch_3 = false;
    bool switch_4 = false; 
    /* Pots */
    float POT1;
    float POT2;
    float POT3;
    float POT4;
    float POT5;
    float POT6;
    float previous_POT2;
    float previous_POT3;

} pedal_attributes; 

typedef struct RCfilter {
    float LP_coeff[2];
    float HP_coeff;
    float alpha;
    float out;
    float LP_out;
    float HP_out;
    float HP_in_prev = 0;
    float HP_out_prev = 0;
    float out_buffer;
    float LP_out_prev;
    float RC;
    uint8_t filterType;
} RCfilter;

/* 
 ---- FILTER SPECS ---- 
Anti aliasing FIR filter

Sampling frequency: 48000 Hz

 -- PASS BAND --
    0 Hz - 10000 Hz
    gain = 1
    desired ripple = 1 dB
    actual ripple = 0.6943232005924238 dB

 -- STOP BAND --
    12000 Hz - 24000 Hz
    gain = 0
    desired attenuation = -60 dB
    actual attenuation = -60.692689098108126 dB
*/

typedef struct FIRfilter {
    float buf[FILTER_TAP_NUM];
    uint8_t bufIndex; 
    float fir_out;
} FIRfilter;

float ANTI_ALIASING_LP_FILTER[FILTER_TAP_NUM] = {
    0.0022085653812701925, 0.00639294176764662, 0.009070412188086173, 0.007062307516343323, -0.0004391254412171878, -0.006948527026922786, -0.005876972991221182, 0.0030006682504004215, 0.009911435908153236, 
    0.005695819855919352, -0.007193020983623251, -0.013995763915948804, -0.004218409015053069, 0.013899534397328285, 0.018605413605505144, -0.00013554015137675498, -0.024322544905887786, -0.023170196994206756, 
    0.00997737554868378, 0.04130800675068791, 0.02705716772487698, -0.0327904564654493, -0.07664310969115254, -0.029674265442697394, 0.11926523549089836, 0.28916704676097094, 0.3639316514267029, 
    0.28916704676097094, 0.11926523549089836, -0.029674265442697394, -0.07664310969115254, -0.0327904564654493, 0.02705716772487698, 0.04130800675068791, 0.00997737554868378, -0.023170196994206756, 
    -0.024322544905887786, -0.00013554015137675498, 0.018605413605505144, 0.013899534397328285, -0.004218409015053069, -0.013995763915948804, -0.007193020983623251, 0.005695819855919352, 
    0.009911435908153236, 0.0030006682504004215, -0.005876972991221182, -0.006948527026922786, -0.0004391254412171878, 0.007062307516343323, 0.009070412188086173, 0.00639294176764662, 0.0022085653812701925
};


pedal_attributes pedalAttribute;
DaisyPetal hw;
Chorus     ch;
Led led1, led2;
RCfilter filter;
FIRfilter fir;
signal_buffer buffer;

void ProcessADC();

/* Clipping functions */
float softClip1(float in);
float hardClip(float in);

/* Filter functions */
void RCfilter_init(RCfilter *filt, float fc_LP, float fc_HP);
void RCfilter_setCutOff(RCfilter *filt, float fc_LP, float fc_HP);
float LP_RCfilter_Update(RCfilter *filt, float input);
float HP_RCfilter_Update(RCfilter *filt, float input); 
void AA_FIR_init(FIRfilter *fir);
float AA_FIR_update(FIRfilter *fir, float in, float preGain);


void pedalControls()
{
    hw.ProcessAllControls();

    /* Footswitch */
    pedalAttribute.FS1_bypass ^= hw.switches[Terrarium::FOOTSWITCH_1].RisingEdge();    
    pedalAttribute.FS2_bypass ^= hw.switches[Terrarium::FOOTSWITCH_2].RisingEdge();  

    /* Switches */
    pedalAttribute.switch_1 ^= hw.switches[Terrarium::SWITCH_1].RisingEdge();

    /* Pots */
    pedalAttribute.POT1 = hw.knob[Terrarium::KNOB_1].Process(); // Volume control
    pedalAttribute.POT2 = hw.knob[Terrarium::KNOB_2].Process(); // LP filter cut off frequency
    pedalAttribute.POT3 = hw.knob[Terrarium::KNOB_3].Process(); // HP filter cut off frequency
    pedalAttribute.POT4 = hw.knob[Terrarium::KNOB_4].Process() * 35; // Overdrive pregain
}

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    pedalControls();
    ProcessADC();

    /* LOWPASS FILTER CUTOFF */
    if (pedalAttribute.POT2 != pedalAttribute.previous_POT2)
    {
        filter.filterType = LOWPASS;
        RCfilter_setCutOff(&filter, 5000.0f * pedalAttribute.POT2, 0.0f);
        pedalAttribute.previous_POT2 = pedalAttribute.POT2;
    }
    // TODO: MOVE TO ProcessADC() function
    /* HIGHPASS FILTER CUTOFF */
    if (pedalAttribute.POT3 != pedalAttribute.previous_POT3)
    {
        filter.filterType = HIGHPASS;
        RCfilter_setCutOff(&filter, 0.0f, 2000.0f * pedalAttribute.POT3);
        pedalAttribute.previous_POT3 = pedalAttribute.POT3;
    }

    for(size_t i = 0; i < size; i++)
    {   
        buffer.in = in[0][i] * (pedalAttribute.POT1*2);

        if (hw.switches[Terrarium::FOOTSWITCH_1].RisingEdge() || hw.switches[Terrarium::FOOTSWITCH_2].RisingEdge()) // Set LEDs
        {    
            led1.Set(!pedalAttribute.FS1_bypass ? 1.0f : 0.0f);
            led2.Set(!pedalAttribute.FS2_bypass ? 1.0f : 0.0f);
            led1.Update(); 
            led2.Update();
        }

         if (!pedalAttribute.FS1_bypass) // filtered distortion
        {
            buffer.lowpass_filtered = LP_RCfilter_Update(&filter, buffer.in);
            buffer.clipped = softClip1(buffer.lowpass_filtered * pedalAttribute.POT1 * 100);
            buffer.highpass_filtered = HP_RCfilter_Update(&filter, buffer.clipped);

            out[0][i] = buffer.highpass_filtered;
        }
        else if (!pedalAttribute.FS2_bypass)
        {
            buffer.highpass_filtered = HP_RCfilter_Update(&filter, buffer.in);

            buffer.clipped = AA_FIR_update(&fir, buffer.highpass_filtered, pedalAttribute.POT4);

            //buffer.lowpass_filtered = LP_RCfilter_Update(&filter, buffer.clipped);

            out[0][i] = buffer.clipped;
        }
        else // Bypass
        {
            out[0][i] = buffer.in;
        }
    }
}

void ProcessADC() 
{
}

/* ----- FIR ANTI-ALIAS FILTER ----- */

void AA_FIR_init(FIRfilter *fir)   
{
    for (uint8_t n = 0; n < FILTER_TAP_NUM; n++) // Reset all buffer indexes
    {
        fir->buf[n] = 0.0f;
    }

    fir->bufIndex = 0.0f;
    fir->fir_out = 0.0f;
}

float overdrive_update(FIRfilter *fir, float in, float preGain)
{
    /* Anti-aliasing lowpass filter */
    fir->buf[fir->bufIndex] = in; // Store latest sample in buffer (circular buffer)
    fir->bufIndex++;

    if (fir->bufIndex == FILTER_TAP_NUM) // Reset index at highest filter tap
    {
        fir->bufIndex = 0.0f;
    }

    // Compute new output sample via convolution
    fir->fir_out = 0.0f;

    uint8_t sumIndex = fir->bufIndex;

    for (uint8_t n = 0; n < FILTER_TAP_NUM; n++)
    {
        if (sumIndex > 0)
        {
            sumIndex--;
        } 
        else 
        {
            sumIndex = FILTER_TAP_NUM - 1;    
        }

        fir->fir_out += ANTI_ALIASING_LP_FILTER[n] * fir->buf[sumIndex];
    }

    /* Overdrive */
    float clipIn = fir->fir_out * preGain;
    float absClipIn = fabs(clipIn);
    float signClipIn = (clipIn >= 0.0f) ? 1.0f : -1.0f;
    float clipOut = 0.0f;

    if (absClipIn < 0.33333f)
    {
        clipOut = 2.0f * clipIn;
    } 
    else if (absClipIn >= 0.33333f && absClipIn < (2.0f * 0.33333f))
    {
        clipOut = signClipIn * (3.0f - (2.0f - 3.0f * absClipIn) * (2.0f - 3.0f * absClipIn)) / 3.0f;
    }
    else
    {
        clipOut = signClipIn;
    }

    if (clipOut > 1.0f)
    {
        clipOut = 1.0f;
    }
    else if (clipOut < -1.0f)
    {
        clipOut = -1.0f;
    }

    return clipOut;
}

/* ----- RC LP/HP FILTER ----- */

void RCfilter_init(RCfilter *filt, float fc_LP, float fc_HP)
{
    RCfilter_setCutOff(filt, fc_LP, fc_HP);
    filt->out = 0.0f;
    filt->out_buffer = 0.0f;
}

void RCfilter_setCutOff(RCfilter *filt, float fc_LP, float fc_HP)
{
    if (filt->filterType == LOWPASS)
    {
        if (fc_LP<50) fc_LP=50; // 50 Hz threshold (sound cuts out around here)
        filt->RC = 1.0f / (PI_2 * fc_LP);
        /* Compute filter coefficients with new cut off frequency */
        filt->LP_coeff[0] = SAMPLE_TIME / (SAMPLE_TIME + filt->RC);
        filt->LP_coeff[1] = filt->RC / (SAMPLE_TIME + filt->RC);
    }
    else if (filt->filterType == HIGHPASS)
    {
        filt->alpha = PI_2 * SAMPLE_TIME * fc_HP;
        filt->HP_coeff = 1.0f / (1.0f + filt->alpha);
    }
}

float LP_RCfilter_Update(RCfilter *filt, float input) 
{
    filt->LP_out = filt->LP_coeff[0] * input + filt->LP_coeff[1] * filt->LP_out_prev;
    filt->LP_out_prev = filt->LP_out;

    /*if (filt->out > 1.0f) 
        filt->out = 1.0f;
    else if (filt->out < -1.0f) 
        filt->out = -1.0f;*/

    return filt->LP_out; 
}

float HP_RCfilter_Update(RCfilter *filt, float input) 
{
    filt->HP_out = filt->HP_coeff * ((input - filt->HP_in_prev) + filt->HP_out_prev);
    filt->HP_out_prev = filt->HP_out;
    filt->HP_in_prev = input; 
    return filt->HP_out;
}


float softClip1(float in)
{
    if(in > 0)
        return 1 - expf(-in);
    return -1 + expf(in);
}

float hardClip(float in)
{
    in = in > 1.f ? 1.f : in;
    in = in < -1.f ? -1.f : in;
    return in;
}

int main(void)
{
    hw.Init();
    hw.SetAudioBlockSize(4);
    float sample_rate = hw.AudioSampleRate();

    led1.Init(hw.seed.GetPin(Terrarium::LED_1),false);
    led2.Init(hw.seed.GetPin(Terrarium::LED_2),false);
    led1.Update();
    led2.Update();
    
    AA_FIR_init(&fir);
    RCfilter_init(&filter, 1000.0f, 1000.0f);

    hw.StartAdc();
    hw.StartAudio(AudioCallback);


    while(1)
    {
        hw.DelayMs(6);
        hw.ClearLeds();
    }
}
