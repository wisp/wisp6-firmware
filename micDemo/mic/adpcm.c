/*****************************************************************************/
/*   ADPCM Encoder and Decoder (based on IMA ADPCM)                          */
/*                                                                           */
/*   Initialization, encoder, and decoder function for ADPCM compression.    */
/*                                                                           */
/*   Texas Instruments Deutschland GmbH                                      */
/*   July 2007, Christian Hernitscheck                                       */
/*   Built with IAR Embedded Workbench Version 3.42A                         */
/*****************************************************************************/

// TI provided associated material:
// https://www.ti.com/lit/an/slaa361a/slaa361a.pdf?ts=1663436233422


signed int PrevSample;   // Predicted sample
int      PrevStepSize; // Index into step size table

// Index changes
const signed char StepSizeAdaption[8] = {
    -1, -1, -1, -1, 2, 4, 6, 8
};

// Quantizer step size lookup table
const int StepSize[89] = {
       7,    8,    9,   10,   11,   12,   13,   14,
      16,   17,   19,   21,   23,   25,   28,   31,
      34,   37,   41,   45,   50,   55,   60,   66,
      73,   80,   88,   97,  107,  118,  130,  143,
     157,  173,  190,  209,  230,  253,  279,  307,
     337,  371,  408,  449,  494,  544,  598,  658,
     724,  796,  876,  963, 1060, 1166, 1282, 1411,
    1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024,
    3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484,
    7132, 7845, 8630, 9493,10442,11487,12635,13899,
   15289,16818,18500,20350,22385,24623,27086,29794,
   32767
};

//-----------------------------------------------------------------------------
// ADPCM Initialization function.
// ADPCM_Init() should be called before starting a new encode oder decode
// sequence.
void ADPCM_Init(void)
{
    PrevSample = 0;
    PrevStepSize = 0;
}

//-----------------------------------------------------------------------------
// ADPCM Encoder function
//   This function converts a 16-bit sample into a 4-bit ADPCM code.
char ADPCM_Encoder(signed int Input)
{
    int code;        // ADPCM code (4-bit code)
    int Se;          // Signal estimate, Output of predictor
    int step;        // Quantizer step size
    int step2;
    signed int d;    // Signal difference between input sample and Se
    signed int dq;   // Quantized difference signal
    int StepSizePTR; // Step size table pointer

    Se = PrevSample; // Restore previous values of signal estimate
    StepSizePTR = PrevStepSize;   // and step size pointer
    step = StepSize[StepSizePTR];

    d = Input - Se;  // calculate difference between sample and signal estimate

    if (d >= 0)
        code = 0;
    else
    {
        code = 8;
        d = -d;
    }

    step2 = step;       // Quantize signal difference into 4-bit ADPCM code
    if (d >= step2)
    {
        code |= 4;
        d -= step2;
    }
    step2 >>= 1;
    if (d >= step2)
    {
        code |= 2;
        d -= step2;
    }
    step2 >>= 1;
    if (d >= step2)
    {
        code |= 1;
    }

    dq = step >> 3;   // Inverse quantize into reconstructed signal
    if (code & 4)
        dq += step;
    if (code & 2)
        dq += step >> 1;
    if (code & 1)
        dq += step >> 2;

    if (code & 8)
        Se -= dq;
    else
        Se += dq;


    if (Se > 4095)   // check for underflow/overflow
        Se = 4095;
    else if (Se < 0)
        Se = 0;


    StepSizePTR += StepSizeAdaption[code & 0x07]; // find new quantizer stepsize

    if (StepSizePTR < 0)   // check for underflow/overflow
        StepSizePTR = 0;
    if (StepSizePTR > 88)
        StepSizePTR = 88;

    PrevSample = Se;        // save signal estimate and step size pointer
    PrevStepSize = StepSizePTR;

    return (code & 0x0f); // return ADPCM code
}


//-----------------------------------------------------------------------------
// ADPCM Decoder function
//   This function converts the 4-bit ADPCM code into a 16-bit sample.
signed int ADPCM_Decoder(char code)
{
    signed int Se;   // Signal estimate, Output of predictor
    int step;        // Quantizer step size
    signed int dq;   // Quantized predicted difference
    int StepSizePTR; // Index into step size table

    Se = PrevSample; // Restore previous values of signal estimate
    StepSizePTR = PrevStepSize;  // and step size pointer
    step = StepSize[StepSizePTR];

    dq = step >> 3;  // Inverse quantize into reconstructed signal
    if (code & 4)
        dq += step;
    if (code & 2)
        dq += step >> 1;
    if (code & 1)
        dq += step >> 2;

    if (code & 8)
        Se -= dq;
    else
        Se += dq;


    if (Se > 4095)   // check for underflow/overflow
        Se = 4095;
    else if (Se < 0)
        Se = 0;


    StepSizePTR += StepSizeAdaption[code & 0x07]; // find new quantizer stepsize

    if (StepSizePTR < 0)   // check for underflow/overflow
        StepSizePTR = 0;
    if (StepSizePTR > 88)
        StepSizePTR = 88;

    PrevSample = Se;  // save signal estimate and step size pointer
    PrevStepSize = StepSizePTR;

    return(Se);     // return 16-bit sample
}