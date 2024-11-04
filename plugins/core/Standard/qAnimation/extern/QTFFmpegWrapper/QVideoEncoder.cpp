#include "QVideoEncoder.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

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
	#include "libavutil/imgutils.h"

	#include "libswscale/swscale.h"
}

//system
#include <assert.h>

struct FFmpegStuffEnc
{
	AVFormatContext* formatContext;
	AVCodecContext* codecContext;
	AVStream* videoStream;
	AVFrame* frame;
	AVPacket* packet;
	SwsContext* swsContext;

	FFmpegStuffEnc()
		: formatContext(nullptr)
		, codecContext(nullptr)
		, videoStream(nullptr)
		, frame(nullptr)
		, packet(nullptr)
		, swsContext(nullptr)
	{}
};

QVideoEncoder::QVideoEncoder(QString filename, int width, int height, unsigned bitrate, int gop, int fps)
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
	if (!m_ff->codecContext)
	{
		assert(false);
		return false;
	}

	assert(!m_ff->frame);
	m_ff->frame = av_frame_alloc();
	if (!m_ff->frame)
	{
		return false;
	}

	m_ff->frame->format = m_ff->codecContext->pix_fmt;
	m_ff->frame->width  = m_ff->codecContext->width;
	m_ff->frame->height = m_ff->codecContext->height;
	//m_ff->frame->pts = 0;

    // allocate the buffers for the frame data
    int ret = av_frame_get_buffer(m_ff->frame, 0);
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
		av_frame_free(&m_ff->frame);
		m_ff->frame = nullptr;
	}
}

bool QVideoEncoder::GetSupportedOutputFormats(std::vector<OutputFormat>& formats, bool ignoreIfNoFileExtension/*=true*/)
{
	try
	{
		// list of all output formats
		void* ofmt_opaque = nullptr;
		while (true)
		{
			const AVOutputFormat* format = av_muxer_iterate(&ofmt_opaque);
			if (format)
			{
				// potentially skip the output formats without any extension (= test formats mostly)
				if (	format->video_codec != AV_CODEC_ID_NONE
					&&	(!ignoreIfNoFileExtension || (format->extensions && format->extensions[0] != 0))
					)
				{
					auto codec = avcodec_find_encoder(format->video_codec);
					if (!codec)
					{
						// failed to find the codec? Silently skip it
						continue;
					}
					if (codec->type != AVMEDIA_TYPE_VIDEO)
					{
						// not a video codec
						continue;
					}

					OutputFormat f;
					{
						f.shortName = format->name;
						f.longName = format->long_name;
						f.extensions = format->extensions;
					}
					formats.push_back(f);
				}
			}
			else
			{
				break;
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	return true;
}

static QString GetAVError(int errnum)
{
	char buffer[256]{ 0 };
	av_make_error_string(buffer, 256, errnum);

	return QString(buffer);
}

bool QVideoEncoder::open(QString formatShortName, QStringList& errors)
{
	if (m_isOpen)
	{
		//the stream is already opened!
		return false;
	}

	if (!isSizeValid())
	{
		errors << "Invalid video size";
		return false;
	}

	const AVOutputFormat* outputFormat = nullptr;
	if (!formatShortName.isEmpty())
	{
		outputFormat = av_guess_format(qPrintable(formatShortName), nullptr, nullptr);
		if (!outputFormat)
		{
			errors << "Could not find output format from short name: " + formatShortName;
		}
	}

	// find the output format
	avformat_alloc_output_context2(&m_ff->formatContext, outputFormat, nullptr, outputFormat ? qPrintable(m_filename) : nullptr);
	if (!m_ff->formatContext)
	{
		if (!outputFormat)
		{
			errors << "Could not deduce output format from file extension: using MPEG";

			avformat_alloc_output_context2(&m_ff->formatContext, nullptr, "mpeg", qPrintable(m_filename));
			if (!m_ff->formatContext)
			{
				errors << "Codec not found";
				return false;
			}
		}
		else
		{
			errors << "Failed to initialize the output context with the specified output format: " + formatShortName;
			return false;
		}
	}

	if (m_ff->formatContext->oformat->flags & AVFMT_NOFILE)
	{
		errors << "Codec is not meant to create a video file";
		return false;
	}

	// get the codec
	AVCodecID codec_id = m_ff->formatContext->oformat->video_codec;
	const AVCodec* pCodec = avcodec_find_encoder(codec_id);
	if (!pCodec)
	{
		errors << "Could not load the codec" + QString(avcodec_get_name(codec_id));
		return false;
	}

	// Allocate the AV packet
	m_ff->packet = av_packet_alloc();
	if (!m_ff->packet)
	{
		errors << "Failed to allocate the AVPacket";
		return false;
	}

	// Add the video stream
	m_ff->videoStream = avformat_new_stream(m_ff->formatContext, pCodec);
	if (!m_ff->videoStream)
	{
		errors << "Failed to allocate the output stream";
		return false;
	}
	m_ff->videoStream->id = m_ff->formatContext->nb_streams - 1;

	// Allocate the codec 'context'
	m_ff->codecContext = avcodec_alloc_context3(pCodec);
	if (!m_ff->codecContext)
	{
		errors << "Failed to allocate an encoding context";
		return false;
	}

	m_ff->codecContext->codec_id = codec_id;
	/* put sample parameters */
	m_ff->codecContext->bit_rate = m_bitrate;
	/* resolution must be a multiple of two */
	m_ff->codecContext->width = m_width;
	m_ff->codecContext->height = m_height;
	/* timebase: This is the fundamental unit of time (in seconds) in terms
	 * of which frame timestamps are represented. For fixed-fps content,
	 * timebase should be 1/framerate and timestamp increments should be
	 * identical to 1. */
	m_ff->videoStream->time_base = { 1, m_fps };
	/* frames per second */
	m_ff->codecContext->time_base = m_ff->videoStream->time_base;
	m_ff->codecContext->gop_size = m_gop;
	m_ff->codecContext->pix_fmt = AV_PIX_FMT_YUV420P;

	//DGM: doesn't really change anything. We only get different warnings if we enabled this :(
	//m_ff->codecContext->max_b_frames = 1;
	//m_ff->codecContext->rc_buffer_size = m_bitrate * 2;
	//m_ff->codecContext->rc_max_rate = m_bitrate;

	if (codec_id == AV_CODEC_ID_H264)
	{
		av_opt_set(m_ff->codecContext->priv_data, "preset", "slow", 0);
	}
	else if (codec_id == AV_CODEC_ID_MPEG1VIDEO)
	{
		/* Needed to avoid using macroblocks in which some coeffs overflow.
		* This does not happen with normal video, it just happens here as
		* the motion of the chroma plane does not match the luma plane. */
		m_ff->codecContext->mb_decision = 2;
	}

	// some formats want stream headers to be separate
	if (m_ff->formatContext->oformat->flags & AVFMT_GLOBALHEADER)
	{
		m_ff->codecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
	}

	// open the codec
	if (avcodec_open2(m_ff->codecContext, pCodec, nullptr) < 0)
	{
		errors << "Could not open the codec";
		return false;
	}

	// allocate and init a re-usable frame
	if (!initFrame())
	{
		errors << "Could not init the internal frame";
		return false;
	}

	// copy the stream parameters to the muxer
	if (avcodec_parameters_from_context(m_ff->videoStream->codecpar, m_ff->codecContext) < 0)
	{
		errors << "Could not copy the stream parameters";
		return false;
	}

	av_dump_format(m_ff->formatContext, 0, qPrintable(m_filename), 1);

	// open the output file
	int ret = avio_open(&m_ff->formatContext->pb, qPrintable(m_filename), AVIO_FLAG_WRITE);
	if (ret < 0)
	{
		errors << QString("Could not open file '%1': %2").arg(m_filename).arg(GetAVError(ret));
		return false;
	}

	// write the stream header, if any
	ret = avformat_write_header(m_ff->formatContext, nullptr);
	if (ret < 0)
	{
		errors << QString("Could not write header: %1").arg(GetAVError(ret));
		return false;
	}

	m_isOpen = true;

	return true;
}

static int write_frame(FFmpegStuffEnc* ff)
{
	if (!ff)
	{
		assert(false);
		return 0;
	}

	// rescale output packet timestamp values from codec to stream timebase
	av_packet_rescale_ts(ff->packet, ff->codecContext->time_base, ff->videoStream->time_base);
	ff->packet->stream_index = ff->videoStream->index;

    // write the compressed frame to the media file
    int ret = av_interleaved_write_frame(ff->formatContext, ff->packet);
	/* pkt is now blank (av_interleaved_write_frame() takes ownership of
	 * its contents and resets pkt), so that no unreferencing is necessary.
	 * This would be different if one used av_write_frame(). */

	return ret;
}

bool QVideoEncoder::close()
{
	if (!m_isOpen)
	{
		return false;
	}

	int ret = avcodec_send_frame(m_ff->codecContext, nullptr);

	// delayed frames?
	while (avcodec_receive_packet(m_ff->codecContext, m_ff->packet) >= 0)
	{
		write_frame(m_ff);
	}

	av_write_trailer(m_ff->formatContext);

	// close the codec
	//avcodec_close(m_ff->codecContext);
	avcodec_free_context(&m_ff->codecContext);
	m_ff->codecContext = nullptr; // just in case

	// free the streams and other data
	freeFrame();
	for(unsigned i = 0; i < m_ff->formatContext->nb_streams; i++)
	{
		av_freep(&m_ff->formatContext->streams[i]);
	}

	av_packet_free(&m_ff->packet);
	m_ff->packet = nullptr; // just in case

	sws_freeContext(m_ff->swsContext);
	m_ff->swsContext = nullptr;

	// close the file
	avio_close(m_ff->formatContext->pb);

	// free the stream
	avformat_free_context(m_ff->formatContext);
	m_ff->formatContext = nullptr;
	m_ff->videoStream = nullptr;

	m_isOpen = false;

	return true;
}

bool QVideoEncoder::encodeImage(const QImage &image, int frameIndex, QString* errorString/*=nullptr*/)
{
	if (!isOpen())
	{
		if (errorString)
		{
			*errorString = "Stream is not opened";
		}
		return false;
	}

	/* when we pass a frame to the encoder, it may keep a reference to it
	 * internally; make sure we do not overwrite it here */
	if (av_frame_make_writable(m_ff->frame) < 0)
	{
		if (errorString)
		{
			*errorString = "Encoder freame is not writable";
		}
		return false;
	}

	// SWS conversion
	if (!convertImage_sws(image, errorString))
	{
		return false;
	}

	// encode the image
	{
		m_ff->frame->pts = frameIndex;

		int ret = avcodec_send_frame(m_ff->codecContext, m_ff->frame);
		if (ret < 0)
		{
			if (errorString)
			{
				*errorString = QString("Error encoding video frame: %1").arg(GetAVError(ret));
			}
			return false;
		}

		while (ret >= 0)
		{
			ret = avcodec_receive_packet(m_ff->codecContext, m_ff->packet);
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
			{
				break;
			}
			else if (ret < 0)
			{
				if (errorString)
				{
					*errorString = QString("Error receiving video frame: %1").arg(GetAVError(ret));
				}
				return false;
			}

			ret = write_frame(m_ff);
			if (ret < 0)
			{
				if (errorString)
				{
					*errorString = QString("Error while writing video frame: %1").arg(GetAVError(ret));
				}
				return false;
			}
		}
	}

	return true;
}

bool QVideoEncoder::convertImage_sws(const QImage &image, QString* errorString/*=nullptr*/)
{
	// Check if the image matches the size
	if (image.width() != m_width || image.height() != m_height)
	{
		if (errorString)
		{
			*errorString = "Wrong image size";
		}
		return false;
	}

	QImage::Format format = image.format();
	if (	format != QImage::Format_RGB32
		&&	format != QImage::Format_ARGB32
		&&	format != QImage::Format_ARGB32_Premultiplied )
	{
		if (errorString)
		{
			*errorString = "Wrong image format";
		}
		return false;
	}

	//Check if context can be reused, otherwise reallocate a new one
	m_ff->swsContext = sws_getCachedContext(m_ff->swsContext,
											m_width,
											m_height,
											AV_PIX_FMT_BGRA,
											m_width,
											m_height,
											AV_PIX_FMT_YUV420P,
											SWS_BICUBIC,
											nullptr,
											nullptr,
											nullptr);

	if (m_ff->swsContext == nullptr)
	{
		if (errorString)
		{
			*errorString = "[SWS] Cannot initialize the conversion context";
		}
		return false;
	}

	int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_BGRA, m_width, m_height, 1);
	if (num_bytes != image.byteCount())
	{
		if (errorString)
		{
			*errorString = "[SWS] Number of bytes mismatch";
		}
		return false;
	}

	const uint8_t* srcSlice[3] { static_cast<const uint8_t*>(image.constBits()), nullptr, nullptr };
	int srcStride[3] { image.bytesPerLine(), 0, 0 };

	sws_scale(	m_ff->swsContext,
				srcSlice,
				srcStride,
				0,
				m_height,
				m_ff->frame->data,
				m_ff->frame->linesize );

	return true;
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

