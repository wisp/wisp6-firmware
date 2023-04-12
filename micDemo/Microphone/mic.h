/*
 * mic.h
 *
 *  Created on: Feb 27, 2023
 *      Author: saffaria
 */

#ifndef MIC_H_
#define MIC_H_


#define CAPTURE_DONE            (0xCA)      // Means CAmera.
#define CAPTURE_REQ             (0x35)      // CAmera bar. 0x35 = !(0xCA)
#define AUDIO_TAG_ID            (0xAD)      // For gui to specify this is in camera mode.
#define AUDIO_SIZE              (32000) // It should be set based on the size of the selected format.
#define AUDIO_DATA_TOBESENT     (32000)
#define SEQUENCE_END_REPEAT     (50)
#define AUDIO_START_ADD         (0x7000)
#define AUDIO_CAPTURE_DONE      (INFO_A)
#define AUDIO_ADDRESS_TOSEND    (INFO_A + 2)
#define AUDIO_LENGTH_SENT       (INFO_A + 4)
#define AUDIO_SEQUENCE_COUNTER  (INFO_A + 6)
#define INFO_A                  (0x1980)


#endif /* MIC_H_ */
