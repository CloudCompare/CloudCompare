//##########################################################################
//#                                                                        #
//#                             MITICAOLIBS                                #
//#                                                                        #
//#    Warning: This computer program is protected by copyright law and    #
//#    international treaties. Unauthorized reproduction or distribution   #
//#    of this program, or any portion of it, may result in severe civil   #
//#    penalties, and will be prosecuted. All rights reserved.             #
//#                                                                        #
//#                        COPYRIGHT: TIMC-IMAG (UJF)                      #
//#                                                                        #
//##########################################################################
//
//*********************** Last revision of this file ***********************
//$Author::                                                                $
//$Rev::                                                                   $
//$LastChangedDate::                                                       $
//**************************************************************************

#include "MarkersFrameBuffer.h"

//Qt
#include <QMutex>
#include <QFileInfo>
#include <QMutexLocker>

//System
#include <assert.h>

//MiticaoDataBase
#include <Log.h>
#include <MarkersFrameBufferBackupThread.h>
#include <LocalizationContext.h>
#include <ConfigurationManager.h>

//Always last!
#include <NewMLP.h>

//-----------------------------------------------------------------------------------------
MarkersFrameBuffer::MarkersFrameBuffer()
	: m_markersStateBuffer(0)
    , m_size(0)
    , m_halfSize(0)
    , m_pos(0)
    , m_full(false)
    , m_mutex(0)
    , m_backupFile(0)
    , m_bufferModified(false)
	, m_pushEnabled(false)
    , m_lastTimestamp(0)
    , m_backupThread(0)
//-----------------------------------------------------------------------------------------
{
    m_mutex = newMLP QMutex(/*QMutex::Recursive*/);
}

//-----------------------------------------------------------------------------------------
MarkersFrameBuffer::~MarkersFrameBuffer()
//-----------------------------------------------------------------------------------------
{
    clear();

    if (m_mutex)
        delete m_mutex;
    m_mutex=0;
}

//-----------------------------------------------------------------------------------------
void MarkersFrameBuffer::clear()
//-----------------------------------------------------------------------------------------
{
    stopBackup();

    if (m_markersStateBuffer)
        delete[] m_markersStateBuffer;

    m_markersStateBuffer = 0;
    m_size         = 0;
    m_halfSize     = 0;

    m_pos               = 0;
    m_lastTimestamp     = 0;
    m_bufferModified    = false;
    m_full              = false;
}

//-----------------------------------------------------------------------------------------
unsigned MarkersFrameBuffer::size() const
//-----------------------------------------------------------------------------------------
{
    return m_size;
}

//-----------------------------------------------------------------------------------------
unsigned MarkersFrameBuffer::fillCount() const
//-----------------------------------------------------------------------------------------
{
    //warning: not thread safe
    return (m_full ? m_size : m_pos);
}

//-----------------------------------------------------------------------------------------
unsigned MarkersFrameBuffer::memSize() const
//-----------------------------------------------------------------------------------------
{
    return m_size * sizeof(MarkersFrame);
}

//-----------------------------------------------------------------------------------------
bool MarkersFrameBuffer::init(unsigned count)
//-----------------------------------------------------------------------------------------
{
	m_pushEnabled = false;
    clear();

	if (count==0)
	{
        Log::Error(QString("[MarkersFrameBuffer::init] Size shouldn't be 0?!"));
		return false;
	}

    if (count&1)
	{
        Log::Warning(QString("[MarkersFrameBuffer::init] Size shouldn't be odd!"));
		++count;
	}
    
	//try to instantiate 'count' MarkersFrame
    m_markersStateBuffer = newMLP MarkersFrame[count];
    if (!m_markersStateBuffer) //not enough memory?
	{
		Log::Warning(QString("[MarkersFrameBuffer::init] Not enough memory!"));
        return false;
	}

    m_size = count;
    m_halfSize = (m_size>>1);
    m_pos = 0;
	m_pushEnabled = true;

    return true;
}

//-----------------------------------------------------------------------------------------
void MarkersFrameBuffer::reset()
//-----------------------------------------------------------------------------------------
{
    //we flush buffer (but we force backup first)
    if (m_bufferModified)
    {
        unsigned start = (m_pos < m_halfSize ? 0 : m_halfSize);
        doSpawnBackupThread(start,m_pos-start,true);
        m_bufferModified = false;
    }

    m_pos           = 0;
    m_full          = false;
    m_lastTimestamp = 0;
}

//-----------------------------------------------------------------------------------------
void MarkersFrameBuffer::pushMarkersFrame(const MarkersFrame& state)
//-----------------------------------------------------------------------------------------
{
    assert(m_size);

	if (!m_pushEnabled)
		return;

	////check time consistency
	//timestamp t = trans.getTimestamp();
	////assert(t>=0);

    m_mutex->lock();

	//if (m_pos || m_full)
	//{
	//	//we check that last transformation is anterior to the one we are trying to push!
	//	if (m_lastTimestamp > t)
	//	{

	//		Log::Warning(QString("[TransBuffer:%1] Last insertion is older than the precedent one! (time loop?)").arg(getName()));

	//		reset();

	//		//if we were backuping elements, we automatically restart backup (in a new file)
	//		if (m_backupFile)
	//		{
	//			stopBackup();
	//			enableBackup();
	//		}
	//	}
	//}
	//m_lastTimestamp = t;

    //we push MarkersFrame on stack
    m_markersStateBuffer[m_pos++] = state;

    m_bufferModified = true;

    //we automatically backup half of the buffer when we reach the end of one half
    if (m_pos == m_halfSize)
    {
        //Log::Print("[TransBuffer] Requested backup of first half");
        doSpawnBackupThread(0,m_halfSize,false);
    }
    else if (m_pos == m_size)
    {
        //Log::Print("[TransBuffer] Requested backup of second half");
        doSpawnBackupThread(m_halfSize,m_halfSize,false);

        //loop
        m_full=true;
        m_pos=0;
    }

    m_mutex->unlock();
}

//-----------------------------------------------------------------------------------------
const MarkersFrame& MarkersFrameBuffer::getMarkersFrameConst(unsigned index) const
//-----------------------------------------------------------------------------------------
{
    return m_markersStateBuffer[(m_pos + index)%m_size];
}

//-----------------------------------------------------------------------------------------
MarkersFrame& MarkersFrameBuffer::getMarkersFrame(unsigned index) const
//-----------------------------------------------------------------------------------------
{
    return m_markersStateBuffer[(m_pos + index)%m_size];
}

//-----------------------------------------------------------------------------------------
bool MarkersFrameBuffer::enableBackup(const QString& filename /*= QString()*/)
//-----------------------------------------------------------------------------------------
{
    if (m_backupFile)
    {
        Log::Warning("[MarkersFrameBuffer::enableBackup] Backup file already opened!");
        return true;
    }

	if (m_backupThread)
		delete m_backupThread;
	m_backupThread=0;

	m_backupFileName = (filename.isEmpty() ? ConfigurationManager::GetDefaultBufferPath()+ConfigurationManager::GenerateTimeStr("localizer_frames",".ult") : filename);

	m_backupFile = fopen(qPrintable(m_backupFileName),"wb");
    if (!m_backupFile)
    {
        Log::Error(QString("[MarkersFrameBuffer::enableBackup] Failed to open/create backup file '%1'!").arg(m_backupFileName),false);
        return false;
    }

    //Data version
    //fwrite(&c_DataVersion, sizeof(short), 1, m_backupFile);

    m_backupThread = newMLP MarkersFrameBufferBackupThread(this, m_backupFile);

    return true;
}

//-----------------------------------------------------------------------------------------
void MarkersFrameBuffer::pause(bool state)
//-----------------------------------------------------------------------------------------
{
	m_pushEnabled = !state;
}

//-----------------------------------------------------------------------------------------
void MarkersFrameBuffer::stopBackup()
//-----------------------------------------------------------------------------------------
{
    //we flush buffer (but we force backup first)
    m_mutex->lock();
    if (m_bufferModified)
    {
        unsigned start = (m_pos < m_halfSize ? 0 : m_halfSize);
		unsigned end = m_pos;
        m_bufferModified = false;
		m_mutex->unlock();
        doSpawnBackupThread(start,end-start,true);
    }
	else m_mutex->unlock();

    if (m_backupFile)
        fclose(m_backupFile);
    m_backupFile = 0;

    if (m_backupThread)
        delete m_backupThread;
    m_backupThread = 0;
}

//-----------------------------------------------------------------------------------------
bool MarkersFrameBuffer::doFullBackup(const QString& filename) const
//-----------------------------------------------------------------------------------------
{
    if (m_pos==0)
    {
        Log::Warning("[MarkersFrameBuffer::doFullBackup] Buffer is empty!");
        return true;
    }

    assert(m_markersStateBuffer);

    FILE* backupFile = fopen(qPrintable(filename),"wb");
    if (!backupFile)
    {
        Log::Error(QString("[MarkersFrameBuffer::doFullBackup] Failed to open/create backup file '%1'!").arg(filename),false);
        return false;
    }

    /*** we dump all valid positions to backup file ***/

    double startTime = Timer::Ticks();

    //Data version
    //fwrite(&c_DataVersion, sizeof(short), 1, backupFile);

    //fast but non-generic way :(
    fwrite(m_markersStateBuffer,sizeof(MarkersFrame),m_pos,m_backupFile);

    //slow but generic way :)
    //MarkersFrame* it = m_markersStateBuffer;
    //for (unsigned i=0;i<m_pos;++i,++it)
    //{
    //    for(unsigned j=0;j<MARKER_ROLES_COUNT;++j)
    //        it->states[j].pos.toFile(m_backupFile);
    //}
    //*/

    double endTime = Timer::Ticks();

    Log::Print(QString("[MarkersFrameBuffer] Successfuly backuped %1 position(s) in %2 ms").arg(m_pos).arg(Timer::ToMsec(endTime-startTime)));

    fclose(backupFile);
    return true;
}

//-----------------------------------------------------------------------------------------
void MarkersFrameBuffer::doSpawnBackupThread(unsigned startIndex, unsigned count, bool waitForCompletion /*= false*/)
//-----------------------------------------------------------------------------------------
{
    if (!m_backupThread)
    {
        //Log::Warning("[MarkersFrameBuffer::doSpawnBackupThread] No backup thread active!");
        return;
    }

    m_backupThread->doBackup(startIndex,count);

    if (waitForCompletion)
        m_backupThread->wait();
}

//-----------------------------------------------------------------------------------------
const MarkersFrame* MarkersFrameBuffer::getConstMarkersFrameArray() const
//-----------------------------------------------------------------------------------------
{
	return m_markersStateBuffer;
}

//-----------------------------------------------------------------------------------------
QSharedPointer<MarkersFrameBuffer> MarkersFrameBuffer::FromBackupBuffer(const QString& filename)
//-----------------------------------------------------------------------------------------
{
	assert(!filename.isEmpty());

    //file size
    long size = QFileInfo(filename).size();

    if ( size == 0 || ((size % sizeof(MarkersFrame)) != 0))
        Log::Warning("[TransBuffer::fromBackupBuffer] Invalid size!");

    //number of transformations in file
    long count = size / sizeof(MarkersFrame);
    Log::Print(QString("[TransBuffer] Found %1 trans. in file '%2'").arg(count).arg(filename));
	if (count<1)
		return QSharedPointer<MarkersFrameBuffer>(0);

    FILE* fp = fopen(qPrintable(filename),"rb");
    if (!fp)
        return QSharedPointer<MarkersFrameBuffer>(0);

    QSharedPointer<MarkersFrameBuffer> buffer(newMLP MarkersFrameBuffer());
    if (!buffer->init(count))
    {
        Log::Warning("[MarkersFrameBuffer::fromBackupBuffer] Not enough memory!");
        fclose(fp);
        return QSharedPointer<MarkersFrameBuffer>(0);
    }

    //Data version
    //short dataVersion;
    //fread(&dataVersion, sizeof(short), 1, fp);

    //fast but non-generic way :(
    fread(buffer->m_markersStateBuffer,sizeof(MarkersFrame),count,fp);

    //slow but generic way :)

    //a more generic way
    //unsigned role(0);
    //MarkersFrame* it = buffer->m_markersStateBuffer;
    //for (long i=0;i<count;++i,++it)
    //{
    //    it->states[ role % MARKER_ROLES_COUNT ].pos.fromFile(fp,dataVersion);
    //    ++role;
    //}
    //*/

    //test
    /*Trans3D* it = buffer->m_transformations;
    unsigned dummy;
    timestamp t;
    for (long i=0;i<count;++i,++it)
    {
    fread(&dummy, sizeof(unsigned), 1, fp);
    fread(it->data(), sizeof(UL_SCALAR), 16, fp);
    fread(&dummy, sizeof(unsigned), 1, fp);
    fread(&t, sizeof(timestamp), 1, fp);
    it->setTimestamp(t);
    MARKER_ROLE role;
    fread(&role, sizeof(MARKER_ROLE), 1, fp);
    fread(&dummy, sizeof(unsigned), 1, fp);
    GLMatrix mat;
    mat.fromFile(fp);
    it->setRelativeGLMat(&mat,role);
    //it->fromFile(fp);
    }
    //*/

    fclose(fp);

    if(count < (long)buffer->m_size)
    {
        buffer->m_full = false;
        buffer->m_pos = count;
    }
    else
        buffer->m_full = true;

    buffer->m_bufferModified = true;
    buffer->check();

    return buffer;
}

//-----------------------------------------------------------------------------------------
bool MarkersFrameBuffer::findNearest(timestamp t, double tolMsec, MARKER_ROLE marker, Trans3D* &trans1, Trans3D* &trans2, unsigned& trans1Index, unsigned& trans2Index) const
//-----------------------------------------------------------------------------------------
{
    trans1 = trans2 = 0;

    //current position, search span
    unsigned first,span;
    double tolTicks = Timer::MsecToTicks(tolMsec);

    m_mutex->lock();
    bool locked = true;

    //end of the buffer?
    if (m_full)
    {
        first = m_pos;
        span = m_size-1;
    }
    else
    {
        first = 0;

        //buffering not yet started
        if (m_pos == 0)
        {
            m_mutex->unlock();
            return false;
        }
        span = m_pos-1;

        //if the buffer is less than half full, we can let the other threads write in it right now 
        //(we should have enough time before the buffer "loops")
        if (span < m_halfSize)
        {
            m_mutex->unlock();
            locked = false;
        }
    }

    bool l_bExactMatch(false);
    unsigned firstFound = first;

    //last buffered position
    Trans3D* l_lastTrans = &((m_markersStateBuffer + ((first+span) % m_size))->states[marker].pos);
    timestamp t_last = l_lastTrans->getTimestamp();

    //first buffered position
    Trans3D* l_firstTrans = &((m_markersStateBuffer + (first % m_size))->states[marker].pos);
    timestamp t_first = l_firstTrans->getTimestamp();

    unsigned int l_nbStep = 0;

    //if all positions in buffer are earlier than request
    if (t_last <= t)
    {
        trans1 = l_lastTrans;
        firstFound = (first+span) % m_size;
    }
    //if all positions in buffer are older than request
    else if(t_first >= t)
    {
        trans2 = l_firstTrans;
        firstFound = ((first-1) % m_size);
    }
    // Binary search
    else
    {
        unsigned spanTemp = span;
        while (spanTemp>1)
        {
            //we look at the middle of the search interval
            bool oddSpan = (spanTemp & 1);
            if (oddSpan)
                ++spanTemp;
            spanTemp >>= 1;

            unsigned middle = ((firstFound + spanTemp) % m_size);

            trans2 = &((m_markersStateBuffer + middle)->states[marker].pos);
            timestamp tm = trans2->getTimestamp();

            l_nbStep++;

            //the value that we search is in the second half of the current search interval
            if (tm < t)
            {
                firstFound = middle;
                trans1 = trans2;
                if (oddSpan)
                    --spanTemp;
                //we don't use anymore the first half of the buffer, so we can let the other threads write in it
                if (locked)
                {
                    m_mutex->unlock();
                    locked = false;
                }
            }
            else if (tm == t)
            {
                trans1 = trans2;
                trans2 = 0;

                if (locked)
                    m_mutex->unlock();

                l_bExactMatch = true;
                break;
            }
        }
    }

    //Log::Warning(QString("[findNearest:BinarySearch] %1 steps").arg(l_nbStep));

    if(trans1)
    {
        trans1 = 0;
        unsigned count;// = m_full ? m_size : span-firstFound;
        if(firstFound - first >= 0)
            count = firstFound - first + 1;
        else
            count = firstFound - first + span + 1;
        for(unsigned i=firstFound % m_size; count > 0 ; i = i==0 ? span : (i-1)%m_size, --count )
        {
            if( t - (m_markersStateBuffer + i)->states[marker].pos.getTimestamp() > tolTicks)
                break;
            if( (m_markersStateBuffer + i)->states[marker].visible )
            {
                trans1 = &((m_markersStateBuffer + i)->states[marker].pos);
                trans1Index = i;
                break;
            }
        }
    }

    if(trans2)
    {
        trans2 = 0;
        unsigned count;// = m_full ? m_size : span-firstFound;
        if(firstFound+1 - first >= 0)
            count = span + firstFound+1 - first + 1;
        else
            count = first - firstFound+1;
        for(unsigned i=(firstFound+1) % m_size; count > 0 ; i = (i+1)%m_size, --count )
        {
            if((m_markersStateBuffer + i)->states[marker].pos.getTimestamp() - t > tolTicks)
                break;
            if( (m_markersStateBuffer + i)->states[marker].visible )
            {
                trans2 = &((m_markersStateBuffer + i)->states[marker].pos);
                trans2Index = i;
                break;
            }
        }
    }

#ifndef NDEBUG
    if(trans2 != NULL && trans1 != NULL)
    {
        assert(trans2 != trans1);
        if (trans1)
            assert(trans1->getTimestamp()<t);
        if (trans2)
            assert(trans2->getTimestamp()>t);
    }
#endif

    if (locked)
        m_mutex->unlock();

    //if( !trans1 && trans2)
    //    trans1 = trans2;
    /*else */if( !trans1 && !trans2 )
        return false;

    return true;
}

//-----------------------------------------------------------------------------------------
bool MarkersFrameBuffer::findNearest(timestamp t, MARKER_ROLE marker, double tolMsec, Trans3D &trans, bool p_bInterpolate, unsigned* transIndex) const
//-----------------------------------------------------------------------------------------
{
    Trans3D *trans1=0, *trans2=0;
    unsigned trans1Index, trans2Index;
    if ( !findNearest(t,tolMsec,marker,trans1,trans2,trans1Index,trans2Index) )
        return false;

	if (!trans2)
	{
		// Here we can't interpolate
        if(p_bInterpolate)
            return false;

        trans = *trans1;
		if (transIndex)
			*transIndex = trans1Index;
	}
	else if (!trans1)
	{
		// Here we can't interpolate
        if(p_bInterpolate)
            return false;

        trans = *trans2;
		if (transIndex)
			*transIndex = trans2Index;
	}
	else
	{
		if(p_bInterpolate)
		{
			//TODO: by default we return the first transformation... why?
			if (transIndex)
				*transIndex = trans1Index;
			return Trans3D::Interpolate(t, trans1, trans2, trans, tolMsec);
		}
		else
		{
			if (t-trans1->getTimestamp()  < trans2->getTimestamp()-t)
			{
				trans = *trans1;
				if (transIndex)
					*transIndex = trans1Index;
			}
			else
			{
				trans = *trans2;
				if (transIndex)
					*transIndex = trans2Index;
			}
		}
	}

    return true;
}

//-----------------------------------------------------------------------------------------
void MarkersFrameBuffer::check()
//-----------------------------------------------------------------------------------------
{
// TODO_API FIXME
//	Log::Print(QString("[TransBuffer] Start checking '%1' ...").arg(getName()));
//
//	unsigned i,count = fillCount();
//	if (count<10)
//	{
//		Log::Warning("[TransBuffer] [Checking] Not enough position to test integrity!");
//		return;
//	}
//
//	double dt,mean=0.0,std=0.0;
//	timestamp t=0,lastT=0,nextT=0;
//
//	lastT = m_transformations[m_pos].getTimestamp();
//	for (i=1;i<count;++i)
//	{
//		t = m_transformations[(m_pos+i)%m_size].getTimestamp();
//		dt = t-lastT;
//		mean += dt;
//		std += dt*dt;
//		lastT = t;
//	}
//
//	mean /= (double)(count-1);
//	std = sqrt(fabs(std/(double)(count-1) - mean*mean));
//
//	Log::Print(QString("[TransBuffer] Mean intervale = %1 ms (std. dev. = %2 µs)").arg(Timer::ToMsec(mean)).arg(Timer::ToMusec(std)));
//
//	//stats
//	unsigned newIndex = 1;
//
//	std::vector<std::pair<unsigned,double>> jumps;
//	unsigned peaks = 0;
//
//	lastT = m_transformations[m_pos].getTimestamp();
//	t = m_transformations[(m_pos+1)%m_size].getTimestamp();
//	dt = t-lastT;
//
//	for (i=1;i<count-1;++i)
//	{
//		nextT = m_transformations[(m_pos+i+1)%m_size].getTimestamp();
//		double dt2 = nextT-t;
//
//		if (dt < 0)
//		{
//			if (dt2 >= -dt)
//			{
//				++peaks;
//			}
//			else
//			{
//				if (i>1)
//				{
//					double dtStart = Timer::ToMsec(t-m_transformations[m_pos].getTimestamp());
//					double dtEnd = Timer::ToMsec(t-m_transformations[(m_pos+count-1)%m_size].getTimestamp());
//					jumps.push_back(std::pair<unsigned,double>(i,Timer::ToMsec(dt)));
//
//					//TEST
//					m_size = i;
//					Trans3D *transBefore=0, *transAfter=0, *newTrans=0;
//					if (findNearest(t, transBefore, transAfter))
//					{
//						//if we find an exactly equivalent transformation!
//						if (transAfter == 0) 
//						{
//							//we test the rest of the buffer
//							unsigned j=i+1;
//							while (j<count)
//							{
//								t = m_transformations[(m_pos+j)%m_size].getTimestamp();
//								if (!findNearest(t, transBefore, transAfter) || (transBefore && transBefore->getTimestamp() != t))
//									break;
//								++j;
//							}
//
//							Log::Warning(QString("[TransBuffer] Shrink buffer from pos #%1 to %2 ...").arg(i+1).arg(j));
//
//							if (j<count)
//							{
//								for (unsigned k=j;k<count;++k)
//									m_transformations[(m_pos+i+k-j)%m_size] = m_transformations[(m_pos+k)%m_size];
//								count -= (j-i);
//							}
//							else
//							{
//								count = i;
//							}
//						}
//					}
//					m_size = count;
//					//*/
//				}
//				else
//				{
//					++peaks; //aberration on first point is considered as a peak
//				}
//			}
//		}
//		
//		lastT=t;
//		t=nextT;
//		dt=dt2;
//	}
//	//last point
//	if (dt < 0)
//	{
//		++peaks; //aberration on last point is considered as a peak
//	}
//
//	if (peaks>0 || !jumps.empty())
//	{
//		Log::Print(QString("[TransBuffer] Detected %1 peaks and %2 negative jumps").arg(peaks).arg(jumps.size()));
//
//#ifdef _DEBUG
//		for (i=0;i<jumps.size();++i)
//			Log::Print(QString("[TransBuffer] Jump #%1 : %2 / dt=%3 ms").arg(i).arg(jumps[i].first).arg(jumps[i].second));
//#endif
//	}
}

//-----------------------------------------------------------------------------------------
MarkersFrameBuffer* MarkersFrameBuffer::extract(double startMuSec, double endMuSec)
//-----------------------------------------------------------------------------------------
{
    unsigned currentSize = fillCount();
    if (currentSize<1)
        return 0;

    unsigned i,j,k,countIn=0;
    bool l_bIsFound;
    double musec;
    for (i=0;i<currentSize;++i)
    {
        j = (i+m_pos+m_size-1)%m_size;
        l_bIsFound = false;
        for(k=0; k<MARKER_ROLES_COUNT && !l_bIsFound;++k)
        {
            if(m_markersStateBuffer[j].states[k].visible)
            {
                musec = Timer::ToMusec(m_markersStateBuffer[j].states[k].pos.getTimestamp());
                l_bIsFound = true;
            }
        }

        assert(l_bIsFound);

        if (musec>=startMuSec && musec<=endMuSec)
            ++countIn;
    }

    if (countIn<1)
        return 0;

    MarkersFrameBuffer* newBuffer = newMLP MarkersFrameBuffer();
    newBuffer->init(countIn);
    for (i=0;i<currentSize;++i)
    {
        j = (i+m_pos+m_size-1)%m_size;
        l_bIsFound = false;
        for(k=0; k<MARKER_ROLES_COUNT && !l_bIsFound;++k)
        {
            if(m_markersStateBuffer[j].states[k].visible)
            {
                musec = Timer::ToMusec(m_markersStateBuffer[j].states[k].pos.getTimestamp());
                l_bIsFound = true;
            }
        }
        if (musec>=startMuSec && musec<=endMuSec)
            newBuffer->pushMarkersFrame(m_markersStateBuffer[j]);
    }

    return newBuffer;
}

//-----------------------------------------------------------------------------------------
QSharedPointer<LocalizationContext> MarkersFrameBuffer::getMarkerContext(timestamp t, double tolMsec, bool p_bInterpolate , MARKER_ROLE marker, MARKER_ROLE reference/* = MARKER_DEFAULT_REF*/)
//-----------------------------------------------------------------------------------------
{
    if (reference == MARKER_DEFAULT_REF)
		reference = LocalizationContext::GetReferenceMarker();

    assert(marker < MARKER_LOCALIZER);
    LocalizationContext* context = newMLP LocalizationContext(marker,reference);

    Trans3D l_transProducer, l_transReference;
    unsigned l_transIndex;
    if( findNearest(t, marker, tolMsec, l_transProducer, p_bInterpolate, &l_transIndex) )
    {
        context->setProducerTrans(&l_transProducer, false);

        if( findNearestNextTo(l_transIndex,t, reference, tolMsec, l_transReference, p_bInterpolate) )
        {
            context->setReferenceTrans(&l_transReference, false);
        }
        else
        {
            delete context;
			context=0;
        }
    }
    else
    {
        delete context;
		context=0;
    }

    return QSharedPointer<LocalizationContext>(context);
}

//-----------------------------------------------------------------------------------------
QSharedPointer<LocalizationContext> MarkersFrameBuffer::getMarkerContextOnLastState(timestamp t, double tolMsec, bool p_bInterpolate, MARKER_ROLE marker, MARKER_ROLE reference/* = MARKER_DEFAULT_REF*/)
//-----------------------------------------------------------------------------------------
{
    if (reference == MARKER_DEFAULT_REF)
        reference = LocalizationContext::GetReferenceMarker();
    
    assert(marker < MARKER_LOCALIZER);
    LocalizationContext* context = newMLP LocalizationContext(marker,reference);

    Trans3D l_transProducer, l_transReference;

    if( findNearestOnLastState(t, marker, tolMsec, l_transProducer, p_bInterpolate) )
    {
        context->setProducerTrans(&l_transProducer, false);

        if( findNearestOnLastState(t, reference, tolMsec, l_transReference, p_bInterpolate) )
        {
            context->setReferenceTrans(&l_transReference, false);
        }
        else
        {
            delete context;
			context=0;
        }
    }
    else
    {
        delete context;
		context=0;
    }

    return QSharedPointer<LocalizationContext>(context);
}

////-----------------------------------------------------------------------------------------
//bool MarkersFrameBuffer::findLatest(MARKER_ROLE marker, double tMin, Trans3D* &trans) const
////-----------------------------------------------------------------------------------------
//{
//    QMutexLocker lock(m_mutex);
//
//    unsigned first,span;
//
//    if (m_full)
//    {
//        first = m_pos;
//        span = m_size-1;
//    }
//    else
//    {
//        first = 0;
//        span = m_pos-1;
//    }
//
//    for(unsigned i = (first+span) % m_size ; i>=0 ; --i)
//    {
//        if( std::abs((m_markersStateBuffer + i)->states[marker].pos.getTimestamp() - t) > tolTicks )
//            return false;
//        else if( (m_markersStateBuffer + i)->states[marker].visible )
//        {
//            trans = &((m_markersStateBuffer + i)->states[marker].pos);
//            return true;
//        }
//    }
//
//    return false;
//}

//-----------------------------------------------------------------------------------------
bool MarkersFrameBuffer::findNearestOnLastState(timestamp t, MARKER_ROLE marker, double tolMsec, Trans3D &trans, bool p_bInterpolate) const
//-----------------------------------------------------------------------------------------
{
    Trans3D *trans1=0, *trans2=0;

    if ( !findNearestOnLastState(t,tolMsec,marker,trans1,trans2) )
        return false;

	if (!trans2)
	{
		trans = *trans1;
	}
	else if (!trans1)
	{
		trans = *trans2;
	}
	else
	{
		if(p_bInterpolate)
		{
			return Trans3D::Interpolate(t, trans1, trans2, trans, tolMsec);
		}
		else
		{
			trans = ( t-trans1->getTimestamp()  < trans2->getTimestamp()-t ? *trans1 : *trans2);
		}
	}

    return true;
}

//-----------------------------------------------------------------------------------------
bool MarkersFrameBuffer::findNearestOnLastState(timestamp t, double tolMsec, MARKER_ROLE marker, Trans3D* &trans1, Trans3D* &trans2) const
//-----------------------------------------------------------------------------------------
{
    double tolTicks = Timer::MsecToTicks(tolMsec);
    unsigned first,span;

    QMutexLocker lock(m_mutex);

    if (m_full)
    {
        first = m_pos;
        span = m_size-1;
    }
    else
    {
        first = 0;
        span = m_pos-1;
    }

    unsigned int l_nbStep = 0;

    trans1=0;
    trans2=0;
    bool l_bExactMatch(false);
    timestamp l_timeTemp;
    unsigned firstFound;

    unsigned count = m_full ? m_size : span-first;
    for(unsigned i=(first+span) % m_size; count > 0 ; i = i==0 ? span : (i-1)%m_size, --count )
    {
        l_timeTemp = (m_markersStateBuffer + i)->states[marker].pos.getTimestamp();
        ++l_nbStep;
        if( t < l_timeTemp && (l_timeTemp - t) <= tolTicks)
        {
            trans2 = &((m_markersStateBuffer + i)->states[marker].pos);
        }
        else if( t > l_timeTemp && (t - l_timeTemp) <= tolTicks)
        {
            trans1 = &((m_markersStateBuffer + i)->states[marker].pos);
            firstFound = i;
            break;
        }
        else if( t == l_timeTemp )
        {
            trans1 = &((m_markersStateBuffer + i)->states[marker].pos);
            trans2 = 0;
            l_bExactMatch = true;
        }
        else if( t > l_timeTemp )
        {
            break;
        }
        
    }

    //Log::Warning(QString("[findNearest:EarliestSearch] %1 steps").arg(l_nbStep));

    if(trans1)
    {
        trans1 = 0;
        unsigned count;// = (span+1) +  //m_full ? m_size : span-firstFound;
        if(firstFound - first >= 0)
            count = firstFound - first + 1;
        else
            count = firstFound - first + span + 1;

        for(unsigned i=firstFound % m_size; count > 0 ; i = i==0 ? span : (i-1)%m_size, --count )
        {
            if( t - (m_markersStateBuffer + i)->states[marker].pos.getTimestamp() > tolTicks)
                break;
            if( (m_markersStateBuffer + i)->states[marker].visible )
            {
                trans1 = &((m_markersStateBuffer + i)->states[marker].pos);
                break;
            }
        }
    }

    if(trans2)
    {
        trans2 = 0;
        unsigned count;// = m_full ? m_size : span-firstFound;
        if(firstFound+1 - first >= 0)
            count = span + firstFound+1 - first + 1;
        else
            count = first - firstFound+1;
        for(unsigned i=(firstFound+1) % m_size; count > 0 ; i = (i+1)%m_size, --count )
        {
            if((m_markersStateBuffer + i)->states[marker].pos.getTimestamp() - t > tolTicks)
                break;
            if( (m_markersStateBuffer + i)->states[marker].visible )
            {
                trans2 = &((m_markersStateBuffer + i)->states[marker].pos);
                break;
            }
        }
    }

    if( !trans1 && trans2)
        trans1 = trans2;
    else if( !trans1 && !trans2 )
        return false;

    #ifndef NDEBUG
    assert(trans2 != trans1);
    if (trans1)
        assert(trans1->getTimestamp()<t);
    if (trans2)
        assert(trans2->getTimestamp()>t);
    #endif

    return true;
}

//-----------------------------------------------------------------------------------------
bool MarkersFrameBuffer::getLastMarkerPosition(Trans3D& p_pos, MARKER_ROLE marker, MARKER_ROLE reference)
//-----------------------------------------------------------------------------------------
{
    if (reference == MARKER_DEFAULT_REF)
        reference = LocalizationContext::GetReferenceMarker();

    QMutexLocker lock(m_mutex);

	unsigned l_pos(0);
	if(m_pos != 0)
		l_pos = m_pos-1;
	else
	{
		if(m_full)
			l_pos = m_size-1;
		else
			return false;
	}

    if(reference == MARKER_LOCALIZER)
    {
        if( (m_markersStateBuffer + l_pos)->states[marker].visible )
            p_pos = (m_markersStateBuffer + l_pos)->states[marker].pos;
        else
            return false;
    }
    else
    {
        if( (m_markersStateBuffer + l_pos)->states[marker].visible &&
            (m_markersStateBuffer + l_pos)->states[reference].visible)
        {
            p_pos = (m_markersStateBuffer + l_pos)->states[reference].pos.inverse() * (m_markersStateBuffer + l_pos)->states[marker].pos;
        }
        else
            return false;
    }

    return true;
}

//-----------------------------------------------------------------------------------------
QSharedPointer<LocalizationContext> MarkersFrameBuffer::getLastMarkerContext(MARKER_ROLE marker, MARKER_ROLE reference)
//-----------------------------------------------------------------------------------------
{
    if (reference == MARKER_DEFAULT_REF)
        reference = LocalizationContext::GetReferenceMarker();

    QMutexLocker lock(m_mutex);

	unsigned l_pos(0);
	if(m_pos != 0)
		l_pos = m_pos-1;
	else
	{
		if(m_full)
			l_pos = m_size-1;
		else
			return QSharedPointer<LocalizationContext>(0);
	}
	
	LocalizationContext* context = newMLP LocalizationContext(marker,reference);

	if( (m_markersStateBuffer + l_pos)->states[marker].visible &&
        (m_markersStateBuffer + l_pos)->states[reference].visible)
    {
        context->setProducerTrans(&((m_markersStateBuffer + l_pos)->states[marker].pos), false);
        context->setReferenceTrans(&((m_markersStateBuffer + l_pos)->states[reference].pos), false);
    }
    else
    {
        delete context;
		context=0;
    }

    return QSharedPointer<LocalizationContext>(context);
}

//-----------------------------------------------------------------------------------------
bool MarkersFrameBuffer::findNearestNextTo(unsigned p_index, timestamp t, double tolMsec, MARKER_ROLE marker, Trans3D* &trans1, Trans3D* &trans2) const
//-----------------------------------------------------------------------------------------
{
    QMutexLocker lock(m_mutex);
    trans1 = trans2 = 0;
    unsigned firstFound, first, span;

    double tolTicks = Timer::MsecToTicks(tolMsec);

    if (m_full)
    {
        first = m_pos;
        span = m_size-1;
    }
    else
    {
        first = 0;
        span = m_pos-1;
    }

    assert( (p_index+first)%m_size <= span);

    timestamp t_index = (m_markersStateBuffer + p_index)->states[marker].pos.getTimestamp();

    if(t_index > t)
    {
        trans2 = &((m_markersStateBuffer + p_index)->states[marker].pos);

        firstFound = (p_index - 1)%m_size;

        if((m_markersStateBuffer + (p_index - 1)%m_size )->states[marker].pos.getTimestamp() <= t)
            trans1 = &((m_markersStateBuffer + p_index)->states[marker].pos);
    }
    else
    {
        trans1 = &((m_markersStateBuffer + p_index)->states[marker].pos);

        firstFound = p_index;

        if((m_markersStateBuffer + (p_index + 1)%m_size )->states[marker].pos.getTimestamp() >= t)
            trans2 = &((m_markersStateBuffer + p_index)->states[marker].pos);
    }

    if(trans1)
    {
        trans1 = 0;
        unsigned count;// = (span+1) +  //m_full ? m_size : span-firstFound;
        if(firstFound - first >= 0)
            count = firstFound - first + 1;
        else
            count = firstFound - first + span + 1;

        for(unsigned i=firstFound % m_size; count > 0 ; i = i==0 ? span : (i-1)%m_size, --count )
        {
            if( t - (m_markersStateBuffer + i)->states[marker].pos.getTimestamp() > tolTicks)
                break;
            if( (m_markersStateBuffer + i)->states[marker].visible )
            {
                trans1 = &((m_markersStateBuffer + i)->states[marker].pos);
                break;
            }
        }
    }

    if(trans2)
    {
        trans2 = 0;
        unsigned count;// = m_full ? m_size : span-firstFound;
        if(firstFound+1  - first >= 0)
            count = span + firstFound+1 - first + 1;
        else
            count = first - firstFound+1;
        for(unsigned i=(firstFound+1) % m_size; count > 0 ; i = (i+1)%m_size, --count )
        {
            if((m_markersStateBuffer + i)->states[marker].pos.getTimestamp() - t > tolTicks)
                break;
            if( (m_markersStateBuffer + i)->states[marker].visible )
            {
                trans2 = &((m_markersStateBuffer + i)->states[marker].pos);
                break;
            }
        }
    }

    //if( !trans1 && trans2)
    //    trans1 = trans2;
    /*else */if( !trans1 && !trans2 )
        return false;

    #ifndef NDEBUG
    assert(trans2 != trans1);
    if (trans1)
        assert(trans1->getTimestamp()<t);
    if (trans2)
        assert(trans2->getTimestamp()>t);
    #endif

    return true;
}

//-----------------------------------------------------------------------------------------
bool MarkersFrameBuffer::findNearestNextTo(unsigned p_index, timestamp t, MARKER_ROLE marker, double tolMsec, Trans3D &trans, bool p_bInterpolate) const
//-----------------------------------------------------------------------------------------
{
    Trans3D *trans1=0, *trans2=0;

	if ( !findNearestNextTo(p_index,t,tolMsec,marker,trans1,trans2) )
        return false;

	if (!trans2)
	{
		// Here we can't interpolate
        if(p_bInterpolate)
            return false;
        trans = *trans1;
	}
	else if (!trans1)
	{
		// Here we can't interpolate
        if(p_bInterpolate)
            return false;
		trans = *trans2;
	}
	else
	{
		if(p_bInterpolate)
		{
			return Trans3D::Interpolate(t, trans1, trans2, trans, tolMsec);
		}
		else
		{
			trans = ( t-trans1->getTimestamp()  < trans2->getTimestamp()-t ) ? *trans1 : *trans2;
		}
	}

    return true;
}

//-----------------------------------------------------------------------------------------
void MarkersFrameBuffer::getLastMarkerFrame(MarkersFrame& p_frame)
//-----------------------------------------------------------------------------------------
{
    QMutexLocker lock(m_mutex);
    if(!m_full && m_pos == 0)
    {
        p_frame = MarkersFrame();
        return;
    }
    p_frame = *(m_markersStateBuffer + m_pos-1);
}

//-----------------------------------------------------------------------------------------
const QString& MarkersFrameBuffer::getBackupFileName()
//-----------------------------------------------------------------------------------------
{
    return m_backupFileName;
}
