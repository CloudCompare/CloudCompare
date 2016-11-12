//Inspired from QTFFmpegWrapper (Daniel Roggen) and the 'muxing.c' example in FFmpeg doc/examples
#ifndef Q_VIDEO_ENCODE_HEADER
#define Q_VIDEO_ENCODE_HEADER

//Qt
#include <QImage>

//FFmpeg variables (we don't want to pollute the client project with FFmpeg includes!)
struct FFmpegStuffEnc;

//! Video encoder based on FFmpeg
class QVideoEncoder
{
public:

	//! Default constructor
	/** \param filename video filename
		\param width video width (must be a multiple of 8)
		\param height video height (must be a multiple of 8)
		\param bitrate bit rate (e.g. 400 000)
		\param gop keyframe interval
		\param fps frame rate
	**/
	QVideoEncoder(	QString filename,
					int width,
					int height,
					unsigned bitrate,
					int gop = 12,
					int fps = 25);
	
	virtual ~QVideoEncoder();

	//! Creates an (empty) video/anmiation file
	/** The codec is automatically guessed from the filename.
		If it can't be guessed this way then MPEG is used by default.
		\return success
	**/
	bool open(QString* errorString = 0);

	//! Returns whether the file is opened or not
	inline bool isOpen() const { return m_isOpen; }

	//! Adds an image to the stream
	virtual bool encodeImage(const QImage& image, int frameIndex, QString* errorString = 0);

	//! Closes the file
	virtual bool close();

protected:

	//stream descriptor
	QString m_filename;
	int m_width;
	int m_height;
	unsigned m_bitrate;
	int m_gop;
	int m_fps;
	bool m_isOpen;

	//! FFmpeg variables
	FFmpegStuffEnc* m_ff;

	//! Returns whether the image size is valid
	bool isSizeValid();

	// Alloc/free a frame
	bool initFrame();
	void freeFrame();

	//! Convert the QImage to the internal YUV format
	/** \warning The QImage is allocated by QT without guarantee about the alignment and bytes per lines.
		It *should* be okay as we make sure the image size is a multiple of 8 bytes however it is not
		guaranteed that sws_scale won't at some point require more bytes per line.
	**/
	bool convertImage_sws(const QImage &image, QString* errorString = 0);
};

#endif //Q_VIDEO_ENCODE_HEADER
