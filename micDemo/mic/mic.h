/*
 * mic.h
 *
 *  Created on: Sep 14, 2022
 *      Author: Rohan Menon
 */

#ifndef MIC_MIC_H_
#define MIC_MIC_H_

void MIC_initialize();
void MIC_capture();

#define AUDIO_PIN               (ADC12INCH_15)

#define AUDIO_START_ADDRESS     (0x7000)
// #define AUDIO_RECORD_SECONDS    (2)
// #define AUDIO_END_ADDRESS       (AUDIO_START_ADDRESS + (AUDIO_RECORD_SECONDS * 7500 / 2))
#define AUDIO_END_ADDRESS       (0xAA98)

#define AUDIO_TAG_ID            (0xAD)
#define CAPTURE_DONE            (0xAD)
#define CAPTURE_REQ             (0x52)

#define SEQUENCE_END_REPEAT		(200)
#define AUDIO_DONE_FLAG         (0xFF)

#define DATA_BYTES_PER_TAG      (10)
//#define AUDIO_CAPTURE_DONE		(INFO_A)
//#define AUDIO_ADDRESS_TOSEND	(INFO_A + 2)
//#define AUDIO_LENGTH_SENT		(INFO_A + 4)
//#define AUDIO_SEQUENCE_COUNTER  (INFO_A + 6)
//#define INFO_A					(0x1980)
#define AUDIO_CAPTURE_DONE      (INFO_B)
#define AUDIO_ADDRESS_TOSEND    (INFO_B + 2)
#define AUDIO_SEQUENCE_COUNTER  (INFO_B + 6)
#define AUDIO_LENGTH_SENT       (INFO_B + 4)
#define INFO_B                  (MEM_MAP_INFOB_START)

#endif /* MIC_MIC_H_ */
