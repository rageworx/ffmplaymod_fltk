#ifndef __WIN32AUDIO_H__
#define __WIN32AUDIO_H__

#ifdef __cplusplus
extern "C"{
#endif /// of _cplusplus

enum audio_type_enum
{
	 AUDIO_TYPE_8      = 0x00f  /**<      0000 0000 1111 Some 8 bit  integer encoding. */
	,AUDIO_TYPE_16     = 0x040  /**<      0000 0100 0000 Some 16 bit integer encoding. */
	,AUDIO_TYPE_24     = 0x4000 /**< 0100 0000 0000 0000 Some 24 bit integer encoding. */
	,AUDIO_TYPE_32     = 0x100  /**<      0001 0000 0000 Some 32 bit integer encoding. */
	,AUDIO_TYPE_SIGNED = 0x080  /**<      0000 1000 0000 Some signed integer encoding. */
	,AUDIO_TYPE_FLOAT  = 0xe00  /**<      1110 0000 0000 Some float encoding. */
	,AUDIO_TYPE_SIGNED_16   = (AUDIO_TYPE_16|AUDIO_TYPE_SIGNED|0x10) /**<           1101 0000 signed 16 bit */
	,AUDIO_TYPE_UNSIGNED_16 = (AUDIO_TYPE_16|0x20)                   /**<           0110 0000 unsigned 16 bit */
	,AUDIO_TYPE_UNSIGNED_8  = 0x01                                   /**<           0000 0001 unsigned 8 bit */
	,AUDIO_TYPE_SIGNED_8    = (AUDIO_TYPE_SIGNED|0x02)               /**<           1000 0010 signed 8 bit */
	,AUDIO_TYPE_ULAW_8      = 0x04                                   /**<           0000 0100 ulaw 8 bit */
	,AUDIO_TYPE_ALAW_8      = 0x08                                   /**<           0000 1000 alaw 8 bit */
	,AUDIO_TYPE_SIGNED_32   = AUDIO_TYPE_32|AUDIO_TYPE_SIGNED|0x1000 /**< 0001 0001 1000 0000 signed 32 bit */
	,AUDIO_TYPE_UNSIGNED_32 = AUDIO_TYPE_32|0x2000                   /**< 0010 0001 0000 0000 unsigned 32 bit */
	,AUDIO_TYPE_SIGNED_24   = AUDIO_TYPE_24|AUDIO_TYPE_SIGNED|0x1000 /**< 0101 0000 1000 0000 signed 24 bit */
	,AUDIO_TYPE_UNSIGNED_24 = AUDIO_TYPE_24|0x2000                   /**< 0110 0000 0000 0000 unsigned 24 bit */
	,AUDIO_TYPE_FLOAT_32    = 0x200                                  /**<      0010 0000 0000 32bit float */
	,AUDIO_TYPE_FLOAT_64    = 0x400                                  /**<      0100 0000 0000 64bit float */
	,AUDIO_TYPE_ANY = ( AUDIO_TYPE_SIGNED_16  | AUDIO_TYPE_UNSIGNED_16 | AUDIO_TYPE_UNSIGNED_8
	                  | AUDIO_TYPE_SIGNED_8   | AUDIO_TYPE_ULAW_8      | AUDIO_TYPE_ALAW_8
	                  | AUDIO_TYPE_SIGNED_32  | AUDIO_TYPE_UNSIGNED_32
	                  | AUDIO_TYPE_SIGNED_24  | AUDIO_TYPE_UNSIGNED_24
	                  | AUDIO_TYPE_FLOAT_32   | AUDIO_TYPE_FLOAT_64 ) /**< Any encoding on the list. */
};

typedef struct audio_output_struct
{
	int     fn;			/* filenumber */
	void*   userptr;	/* driver specific pointer */

	/* Callbacks */
	int  (*open)(struct audio_output_struct *);
	int  (*get_formats)(struct audio_output_struct *);
	int  (*write)(struct audio_output_struct *, unsigned char *,int);
	void (*flush)(struct audio_output_struct *);
	int  (*close)(struct audio_output_struct *);
	int  (*deinit)(struct audio_output_struct *);
	int  (*getbuffersize)(void);

	char*   device;	    /* device name */
	int     flags;	    /* some bits; namely headphone/speaker/line */
	long    rate;		/* sample rate */
	long    gain;		/* output gain */
	int     channels;	/* number of channels */
	int     format;		/* format flags */
	int     is_open;	/* something opened? */
	int     auxflags;   /* For now just one: quiet mode (for probing). */
} audio_output_t;

int init_audio(audio_output_t* ao);

typedef void txfermem;

#ifdef __cplusplus
};
#endif /// of _cplusplus

#endif /// of __WIN32AUDIO_H__
