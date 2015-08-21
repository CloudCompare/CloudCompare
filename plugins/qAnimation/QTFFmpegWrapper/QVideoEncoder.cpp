#include "QVideoEncoder.h"

//FFmpeg
extern "C"
{
	#include "libavcodec/avcodec.h"
	#include "libavformat/avformat.h"
	#include "libavformat/avio.h"

	#include "libavutil/mathematics.h"
	#include "libavutil/opt.h"
	#include "libavutil/error.h"
	#include "libavutil/rational.h"
	#include "libavutil/frame.h"
	#include "libavutil/avstring.h"

	#include "libswscale/swscale.h"
}

//system
#include <assert.h>

struct FFmpegStuffEnc
{
	AVFormatContext *formatContext;
	AVCodecContext *codexContext;
	AVStream *videoStream;
	AVFrame *frame;
	SwsContext *swsContext;

	FFmpegStuffEnc()
		: formatContext(0)
		, codexContext(0)
		, videoStream(0)
		, frame(0)
		, swsContext(0)
	{}
};

QVideoEncoder::QVideoEncoder(QString filename, unsigned width, unsigned height, unsigned bitrate, unsigned gop, unsigned fps)
	: m_filename(filename)
	, m_width(width)
	, m_height(height)
	, m_bitrate(bitrate)
	, m_gop(gop)
	, m_fps(fps)
	, m_isOpen(false)
	, m_ff(new FFmpegStuffEnc)
{
}

QVideoEncoder::~QVideoEncoder()
{
	close();

	if (m_ff)
	{
		delete m_ff;
		m_ff = 0;
	}
}

bool QVideoEncoder::isSizeValid()
{
	return (m_width % 8) == 0 && (m_height % 8) == 0;
}

bool QVideoEncoder::initFrame()
{
	assert(!m_ff->frame);

	m_ff->frame = av_frame_alloc();
	if (!m_ff->frame)
	{
		return false;
	}

	m_ff->frame->format = m_ff->codexContext->pix_fmt;
	m_ff->frame->width  = m_ff->codexContext->width;
	m_ff->frame->height = m_ff->codexContext->height;

    /* allocate the buffers for the frame data */
    int ret = av_frame_get_buffer(m_ff->frame, 32);
    if (ret < 0)
	{
        fprintf(stderr, "Could not allocate frame data.\n");
		return false;
    }

	return true;
}

void QVideoEncoder::freeFrame()
{
	if (m_ff->frame)
	{
		av_free(m_ff->frame);
		m_ff->frame = 0;
	}
}

bool QVideoEncoder::open(QString* errorString/*=0*/)
{
	if (m_isOpen)
	{
		//the stream is already opened!
		return false;
	}

	if (!isSizeValid())
	{
		if (errorString)
			*errorString = "Invalid video size";
		return false;
	}

	//Initialize libavcodec, and register all codecs and formats
	av_register_all();

	// find the output format
	avformat_alloc_output_context2(&m_ff->formatContext, NULL, NULL, qPrintable(m_filename));
	if (!m_ff->formatContext)
	{
		if (errorString)
			*errorString = "Could not deduce output format from file extension: using MPEG";
		
		avformat_alloc_output_context2(&m_ff->formatContext, NULL, "mpeg", qPrintable(m_filename));
		if (!m_ff->formatContext)
		{
			if (errorString)
				*errorString = "Codec not found";
			return false;
		}
	}

	// get the codec
	AVCodecID codec_id = m_ff->formatContext->oformat->video_codec;
	//codec_id = AV_CODEC_ID_MPEG1VIDEO;
	//codec_id = AV_CODEC_ID_H264;
	AVCodec *pCodec = avcodec_find_encoder(codec_id);
	if (!pCodec)
	{
		if (errorString)
			*errorString = "Could not load the codec";
		return false;
	}
	m_ff->codexContext = avcodec_alloc_context3(pCodec);

	// Add the video stream
	m_ff->videoStream = avformat_new_stream(m_ff->formatContext, pCodec);
	if (!m_ff->videoStream )
	{
		if (errorString)
			*errorString = "Failed to allocate the output stream";
		return false;
	}
	m_ff->videoStream->id = m_ff->formatContext->nb_streams-1;
	m_ff->codexContext = m_ff->videoStream->codec;

	/* put sample parameters */
	m_ff->codexContext->bit_rate = m_bitrate;
	/* resolution must be a multiple of two */
	m_ff->codexContext->width = m_width;
	m_ff->codexContext->height = m_height;
	/* frames per second */
	m_ff->codexContext->time_base.num = 1;
	m_ff->codexContext->time_base.den = m_fps;
	m_ff->codexContext->gop_size = m_gop;
	m_ff->codexContext->max_b_frames = 1;
	m_ff->codexContext->pix_fmt = PIX_FMT_YUV420P;

	if (codec_id == AV_CODEC_ID_H264)
	{
		av_opt_set(m_ff->codexContext->priv_data, "preset", "slow", 0);
	}
	else if (codec_id == AV_CODEC_ID_MPEG1VIDEO)
	{
		/* Needed to avoid using macroblocks in which some coeffs overflow.
		* This does not happen with normal video, it just happens here as
		* the motion of the chroma plane does not match the luma plane. */
		m_ff->codexContext->mb_decision = 2;
	}

	//some formats want stream headers to be separate
	if (m_ff->formatContext->oformat->flags & AVFMT_GLOBALHEADER)
	{
		m_ff->codexContext->flags |= CODEC_FLAG_GLOBAL_HEADER;
	}

	//av_dump_format(m_ff->formatContext, 0, fileName.toStdString().c_str(), 1);

	// open the codec
	if (avcodec_open2(m_ff->codexContext, pCodec, 0) < 0)
	{
		if (errorString)
			*errorString = "Could not open the codec";
		return false;
	}

	// Allocate the YUV frame
	if (!initFrame())
	{
		if (errorString)
			*errorString = "Could not init the internal frame";
		return false;
	}

	if (avio_open(&m_ff->formatContext->pb, qPrintable(m_filename), AVIO_FLAG_WRITE) < 0)
	{
		if (errorString)
			*errorString = QString("Could not open '%1'").arg(m_filename);
		return false;
	}

	avformat_write_header(m_ff->formatContext, NULL);

	m_isOpen = true;

	return true;
}

bool QVideoEncoder::close()
{
	if (!m_isOpen)
	{
		return false;
	}

	av_write_trailer(m_ff->formatContext);

	// close the codec
	avcodec_close(m_ff->videoStream->codec);

	// free the streams and other data
	freeFrame();
	for(unsigned i = 0; i < m_ff->formatContext->nb_streams; i++)
	{
		av_freep(&m_ff->formatContext->streams[i]->codec);
		av_freep(&m_ff->formatContext->streams[i]);
	}

	// close the file
	avio_close(m_ff->formatContext->pb);

	// free the stream
	av_free(m_ff->formatContext);

	m_isOpen = false;

	return true;
}

static int write_frame(AVFormatContext *fmt_ctx, const AVRational *time_base, AVStream *st, AVPacket *pkt)
{
    // rescale output packet timestamp values from codec to stream timebase
    av_packet_rescale_ts(pkt, *time_base, st->time_base);
    pkt->stream_index = st->index;

    // write the compressed frame to the media file
    return av_interleaved_write_frame(fmt_ctx, pkt);
}

bool QVideoEncoder::encodeImage(const QImage &image, QString* errorString/*=0*/)
{
	if (!isOpen())
	{
		if (errorString)
			*errorString = "Stream is not opened";
		return false;
	}

	// SWS conversion
	if (!convertImage_sws(image, errorString))
	{
		return false;
	}

	AVPacket pkt = { 0 };
	av_init_packet(&pkt);

	// encode the image
	int got_packet = 0;
	{
		int ret = avcodec_encode_video2(m_ff->codexContext, &pkt, m_ff->frame, &got_packet);
		if (ret < 0)
		{
			char errorStr[AV_ERROR_MAX_STRING_SIZE] = {0};
			av_make_error_string(errorStr, AV_ERROR_MAX_STRING_SIZE, ret);
			if (errorString)
				*errorString = QString("Error encoding video frame: %1").arg(errorStr);
			return false;
		}
	}

	if (got_packet)
	{
		int ret = write_frame(m_ff->formatContext, &m_ff->codexContext->time_base, m_ff->videoStream, &pkt);
		if (ret < 0)
		{
			char errorStr[AV_ERROR_MAX_STRING_SIZE] = {0};
			av_make_error_string(errorStr, AV_ERROR_MAX_STRING_SIZE, ret);
			if (errorString)
				*errorString = QString("Error while writing video frame: %1").arg(errorStr);
			return false;
		}
	}
	
	return true;
}

bool QVideoEncoder::convertImage_sws(const QImage &image, QString* errorString/*=0*/)
{
	// Check if the image matches the size
	if (image.width() != m_width || image.height() != m_height)
	{
		if (errorString)
			*errorString = "Wrong image size";
		return false;
	}
	
	if (	image.format() != QImage::Format_RGB32
		&&	image.format() != QImage::Format_ARGB32 )
	{
		if (errorString)
			*errorString = "Wrong image format";
		return false;
	}

	//Check if context can be reused, otherwise reallocate a new one
	m_ff->swsContext = sws_getCachedContext(	m_ff->swsContext,
													m_width,
													m_height,
													PIX_FMT_BGRA,
													m_width,
													m_height,
													PIX_FMT_YUV420P,
													SWS_BICUBIC,
													NULL,
													NULL,
													NULL);

	if (m_ff->swsContext == NULL)
	{
		if (errorString)
			*errorString = "[SWS] Cannot initialize the conversion context";
		return false;
	}

	int num_bytes = avpicture_get_size(PIX_FMT_BGRA, m_width, m_height);
	if (num_bytes != image.byteCount())
	{
		if (errorString)
			*errorString = "[SWS] Number of bytes mismatch";
		return false;
	}

	uint8_t *srcSlice[3] = { (uint8_t*)image.bits(), 0, 0 };
	int srcStride[3] = { image.bytesPerLine(), 0, 0 };

	sws_scale(	m_ff->swsContext,
						srcSlice,
						srcStride,
						0,
						m_height,
						m_ff->frame->data,
						m_ff->frame->linesize );

	return true;
}

