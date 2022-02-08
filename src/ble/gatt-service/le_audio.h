/*
 * Copyright (C) 2022 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

/**
 * @title Volume Offset Control Service Server
 * 
 */

#ifndef LE_AUDIO_H
#define LE_AUDIO_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

// Generic Audio/Audio Location Definitions/Bitmap
#define LEA_AUDIO_LOCATION_NOT_ALLOWED             0x00000000
#define LEA_AUDIO_LOCATION_FRONT_LEFT              0x00000001
#define LEA_AUDIO_LOCATION_FRONT_RIGHT             0x00000002
#define LEA_AUDIO_LOCATION_FRONT_CENTER            0x00000004
#define LEA_AUDIO_LOCATION_LOW_FREQUENCY_EFFECTS1  0x00000008
#define LEA_AUDIO_LOCATION_BACK_LEFT               0x00000010
#define LEA_AUDIO_LOCATION_BACK_RIGHT              0x00000020
#define LEA_AUDIO_LOCATION_FRONT_LEFT_OF_CENTER    0x00000040
#define LEA_AUDIO_LOCATION_FRONT_RIGHT_OF_CENTER   0x00000080
#define LEA_AUDIO_LOCATION_BACK_CENTER             0x00000100
#define LEA_AUDIO_LOCATION_LOW_FREQUENCY_EFFECTS2  0x00000200
#define LEA_AUDIO_LOCATION_SIDE_LEFT               0x00000400
#define LEA_AUDIO_LOCATION_SIDE_RIGHT              0x00000800
#define LEA_AUDIO_LOCATION_TOP_FRONT_LEFT          0x00001000
#define LEA_AUDIO_LOCATION_TOP_FRONT_RIGHT         0x00002000
#define LEA_AUDIO_LOCATION_TOP_FRONT_CENTER        0x00004000
#define LEA_AUDIO_LOCATION_TOP_CENTER              0x00008000
#define LEA_AUDIO_LOCATION_TOP_BACK_LEFT           0x00010000
#define LEA_AUDIO_LOCATION_TOP_BACK_RIGHT          0x00020000
#define LEA_AUDIO_LOCATION_TOP_SIDE_LEFT           0x00040000
#define LEA_AUDIO_LOCATION_TOP_SIDE_RIGHT          0x00080000
#define LEA_AUDIO_LOCATION_TOP_BACK_CENTER         0x00100000
#define LEA_AUDIO_LOCATION_BOTTOM_FRONT_CENTER     0x00200000
#define LEA_AUDIO_LOCATION_BOTTOM_FRONT_LEFT       0x00400000
#define LEA_AUDIO_LOCATION_BOTTOM_FRONT_RIGHT      0x00800000
#define LEA_AUDIO_LOCATION_FRONT_LEFT_WIDE         0x01000000
#define LEA_AUDIO_LOCATION_FRONT_RIGHT_WIDE        0x02000000
#define LEA_AUDIO_LOCATION_LEFT_SURROUND           0x04000000
#define LEA_AUDIO_LOCATION_RIGHT_SURROUND          0x08000000
#define LEA_AUDIO_LOCATION_RFU                     0xF0000000

// Generic Audio/Context Type
#define LEA_CONTEXT_TYPE_PROHIBITED                0x0000
#define LEA_CONTEXT_TYPE_UNSPECIFIED               0x0001
#define LEA_CONTEXT_TYPE_CONVERSATIONAL            0x0002 // Conversation between humans, for example, in telephony or video calls, including traditional cellular as well as VoIP and Push-to-Talk
#define LEA_CONTEXT_TYPE_MEDIA                     0x0004 // Media, for example, music playback, radio, podcast or movie soundtrack, or tv audio
#define LEA_CONTEXT_TYPE_GAME                      0x0008 // Audio associated with video gaming, for example gaming media; gaming effects; music and in-game voice chat between participants; or a mix of all the above
#define LEA_CONTEXT_TYPE_INSTRUCTIONAL             0x0010 // Instructional audio, for example, in navigation, announcements, or user guidance
#define LEA_CONTEXT_TYPE_VOICE_ASSISTANTS          0x0020 // Man-machine communication, for example, with voice recognition or virtual assistants
#define LEA_CONTEXT_TYPE_LIVE                      0x0040 // Live audio, for example, from a microphone where audio is perceived both through a direct acoustic path and through an LE Audio Stream
#define LEA_CONTEXT_TYPE_SOUND_EFFECTS             0x0080 // Sound effects including keyboard and touch feedback; menu and user interface sounds; and other system sounds
#define LEA_CONTEXT_TYPE_NOTIFICATIONS             0x0100 // Notification and reminder sounds; attention-seeking audio, for example, in beeps signaling the arrival of a message
#define LEA_CONTEXT_TYPE_RINGTONE                  0x0200 // Alerts the user to an incoming call, for example, an incoming telephony or video call, including traditional cellular as well as VoIP and Push-to-Talk
#define LEA_CONTEXT_TYPE_ALERTS                    0x0400 // Alarms and timers; immediate alerts, for example, in a critical battery alarm, timer expiry or alarm clock, toaster, cooker, kettle, microwave, etc.
#define LEA_CONTEXT_TYPE_EMERGENCY_ALARM           0x0800 //Emergency alarm Emergency sounds, for example, fire alarms or other urgent alerts
#define LEA_CONTEXT_TYPE_RFU                       0xF000

typedef enum {
    LEA_CODEC_SPECIFIC_CAPABILITY_TYPE_SAMPLING_FREQUENCY = 0x01,
    LEA_CODEC_SPECIFIC_CAPABILITY_TYPE_FRAME_DURATION,
    LEA_CODEC_SPECIFIC_CAPABILITY_TYPE_AUDIO_CHANNEL_ALLOCATION,
    LEA_CODEC_SPECIFIC_CAPABILITY_TYPE_OCTETS_PER_CODEC_FRAME,
    LEA_CODEC_SPECIFIC_CAPABILITY_TYPE_CODEC_FRAME_BLOCKS_PER_SDU,
} lea_codec_specific_capability_type_t;

typedef enum {
    LEA_CODEC_FRAME_DURATION_7_5_MS = 0x00,
    LEA_CODEC_FRAME_DURATION_10_MS = 0x01
} lea_codec_frame_duration_t;

typedef enum {
    LEA_CODEC_SAMPLING_FREQUENCY_8000_HZ   = 0x01,
    LEA_CODEC_SAMPLING_FREQUENCY_11025_HZ  = 0x02,
    LEA_CODEC_SAMPLING_FREQUENCY_16000_HZ  = 0x03,
    LEA_CODEC_SAMPLING_FREQUENCY_22050_HZ  = 0x04,
    LEA_CODEC_SAMPLING_FREQUENCY_24000_HZ  = 0x05,
    LEA_CODEC_SAMPLING_FREQUENCY_32000_HZ  = 0x06,
    LEA_CODEC_SAMPLING_FREQUENCY_44100_HZ  = 0x07,
    LEA_CODEC_SAMPLING_FREQUENCY_48000_HZ  = 0x08,
    LEA_CODEC_SAMPLING_FREQUENCY_88200_HZ  = 0x09,
    LEA_CODEC_SAMPLING_FREQUENCY_96000_HZ  = 0x0A,
    LEA_CODEC_SAMPLING_FREQUENCY_176400_HZ = 0x0B,
    LEA_CODEC_SAMPLING_FREQUENCY_192000_HZ = 0x0C,
    LEA_CODEC_SAMPLING_FREQUENCY_384000_HZ = 0x0D
} lea_codec_sampling_frequency_t;

// struct for codec id
typedef struct {
    hci_audio_coding_format_t coding_format;
    uint16_t company_id;
    uint16_t vendor_specific_codec_id;
} lea_codec_id_t;

typedef enum {
    LEA_PA_SYNC_STATE_NOT_SYNCHRONIZED_TO_PA = 0x00,
    LEA_PA_SYNC_STATE_SYNCINFO_REQUEST,
    LEA_PA_SYNC_STATE_SYNCHRONIZED_TO_PA,
    LEA_PA_SYNC_STATE_FAILED_TO_SYNCHRONIZE_TO_PA,
    LEA_PA_SYNC_STATE_NO_PAST,
    LEA_PA_SYNC_STATE_RFU
} lea_pa_sync_state_t;

typedef enum {
    LEA_BIG_ENCRYPTION_NOT_ENCRYPTED = 0x00,
    LEA_BIG_ENCRYPTION_BROADCAST_CODE_REQUIRED,
    LEA_BIG_ENCRYPTION_DECRYPTING,
    LEA_BIG_ENCRYPTION_BAD_CODE,
    LEA_BIG_ENCRYPTION_RFU
} lea_big_encryption_t;


#if defined __cplusplus
}
#endif

#endif

