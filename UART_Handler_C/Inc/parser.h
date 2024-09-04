/*
 * parser.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_PARSER_H_
#define INC_PARSER_H_

#include <stdint.h>


#define POS_RES_RX ( 0.030518043793393 )
#define POS_RES_TX ( 32.76750000000000 )

#define VEL_RES_RX ( 0.003051804379339 )
#define VEL_RES_TX ( 327.6750000000000 )

#define ACC_RES_RX ( 0.006103608758679 )
#define ACC_RES_TX ( 163.8375000000000 )

#define ATT_RES_RX ( 0.00004793762111849 )
#define ATT_RES_TX ( 20860.4427309466000 )

#define ANV_RES_RX ( 0.00047937621118486 )
#define ANV_RES_TX ( 2086.04427309466000 )

#define ACT_RES_RX ( 0.001525902189670 )
#define ACT_RES_TX ( 655.3500000000000 )

#define THC_RES_RX ( 0.001525902189670 )
#define THC_RES_TX ( 655.3500000000000 )

#define THR_RES_RX ( 0.152590218966964 )
#define THR_RES_TX ( 6.553500000000000 )

#define ALT_RES_RX ( 0.015259021896696 )
#define ALT_RES_TX ( 65.53500000000000 )

#define LAT_RES_RX ( 0.0000000419095158674544 )
#define LAT_RES_TX ( 23860929.416666700000000 )

#define LON_RES_RX ( 0.0000000838190317349087 )
#define LON_RES_TX ( 11930464.708333300000000 )


#define _CONVERT_F64_2_U08 ( 0 )
#define _CONVERT_F32_2_U08 ( 1 )
#define _CONVERT_I32_2_U08 ( 2 )
#define _CONVERT_I16_2_U08 ( 3 )
#define _CONVERT_U32_2_U08 ( 4 )
#define _CONVERT_U16_2_U08 ( 5 )

typedef union {   int8_t value; uint8_t input;    } UNION_PREDEF_U08_I08;
typedef union {  int16_t value; uint8_t input[2]; } UNION_PREDEF_U08_I16;
typedef union { uint16_t value; uint8_t input[2]; } UNION_PREDEF_U08_U16;
typedef union {  int32_t value; uint8_t input[4]; } UNION_PREDEF_U08_I32;
typedef union { uint32_t value; uint8_t input[4]; } UNION_PREDEF_U08_U32;
typedef union {    float value; uint8_t input[4]; } UNION_PREDEF_U08_F32;
typedef union {   double value; uint8_t input[8]; } UNION_PREDEF_U08_F64;

extern UNION_PREDEF_U08_I08 UNION_U08_I08;
extern UNION_PREDEF_U08_I16 UNION_U08_I16;
extern UNION_PREDEF_U08_U16 UNION_U08_U16;
extern UNION_PREDEF_U08_I32 UNION_U08_I32;
extern UNION_PREDEF_U08_U32 UNION_U08_U32;
extern UNION_PREDEF_U08_F32 UNION_U08_F32;
extern UNION_PREDEF_U08_F64 UNION_U08_F64;

void Compress_F64_2_U08( double   value, uint8_t *storage );
void Compress_F32_2_U08( float    value, uint8_t *storage );
void Compress_I32_2_U08( int32_t  value, uint8_t *storage );
void Compress_I16_2_U08( int16_t  value, uint8_t *storage );
void Compress_U32_2_U08( uint32_t value, uint8_t *storage );
void Compress_U16_2_U08( uint16_t value, uint8_t *storage );

void Compression_2_U08( double *input, uint8_t *storage, uint8_t type, double resolution );

void Extract_U08_2_F64( double   *value, uint8_t *source );
void Extract_U08_2_F32( float    *value, uint8_t *source );
void Extract_U08_2_I32( int32_t  *value, uint8_t *source );
void Extract_U08_2_I16( int16_t  *value, uint8_t *source );
void Extract_U08_2_U32( uint32_t *value, uint8_t *source );
void Extract_U08_2_U16( uint16_t *value, uint8_t *source );

void Extraction_2_F64( double *value, uint8_t *source, uint8_t type, double resolution );
#endif /* INC_PARSER_H_ */
