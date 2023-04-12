/*
 * cam.h
 *
 *  Created on: Feb 27, 2023
 *      Author: saffaria
 */

#ifndef CAM_H_
#define CAM_H_


//Camera globals---------------------------------------------------------------------------------------------------------------------//
#define CAPTURE_DONE            (0xCA)      // Means CAmera.
#define CAPTURE_REQ             (0x35)      // CAmera bar. 0x35 = !(0xCA)
#define CAMERA_TAG_ID           (0xCA)      // For gui to specify this is in camera mode.
#define ROWS                    (122)
#define COLUMNS                 (162)   // QQVGA format: 122*162
#define IMAGE_SIZE              (19764) // It should be set based on the size of the selected format.
#define IMAGE_DATA_TOBESENT     (19764) // 162*122 bytes
#define CAM_POWER_UP_CYCLE      (5000) // May be vary for each specific type of cameras.
#define CAM_STAB_RECONF         (0) // Based on the amount of energy provided we should set it to as high as possible.
#define SEQUENCE_END_REPEAT     (50)
#define IMAGE_START_ADD         (0x7000)
#define IMAGE_CAPTURE_DONE      (INFO_A)
#define IMAGE_ADDRESS_TOSEND    (INFO_A + 2)
#define IMAGE_LENGTH_SENT       (INFO_A + 4)
#define IMAGE_SEQUENCE_COUNTER  (INFO_A + 6)
#define INFO_A                  (0x1980)




#endif /* CAM_H_ */
