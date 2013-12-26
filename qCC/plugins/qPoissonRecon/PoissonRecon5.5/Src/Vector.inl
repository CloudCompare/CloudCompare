/*
Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution. 

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#ifndef __VECTORIMPL_HPP
#define __VECTORIMPL_HPP

////////////
// Vector //
////////////
template<class T>
Vector<T>::Vector( void )
{
	m_N = 0;
	m_pV = NullPointer< T >();
}
template< class T >
Vector< T >::Vector( const Vector<T>& V )
{
	m_N = 0;
	m_pV = NullPointer< T >();
	Resize( V.m_N );
	memcpy( m_pV , V.m_pV , m_N*sizeof(T) );
}
template<class T>
Vector<T>::Vector( size_t N )
{
	m_N=0;
	m_pV = NullPointer< T >();
	Resize(N);
}
template<class T>
void Vector<T>::Resize( size_t N )
{
	if( m_N!=N )
	{
		if( m_N ) DeletePointer( m_pV );
		m_N = N;
		m_pV = NewPointer< T >( N );
	}
	if( N ) memset( m_pV , 0 , N*sizeof(T) );
}

template<class T>
Vector<T>::Vector( size_t N, ConstPointer( T ) pV )
{
	Resize(N);
	memcpy( m_pV, pV, N*sizeof(T) );
}
template<class T>
Vector<T>::~Vector(){Resize(0);}
template<class T>
Vector<T>& Vector<T>::operator = (const Vector& V)
{
	Resize(V.m_N);
	memcpy( m_pV, V.m_pV, m_N*sizeof(T) );
	return *this;
}
template<class T>
size_t Vector<T>::Dimensions() const{return m_N;}
template<class T>
void Vector<T>::SetZero(void){for (size_t i=0; i<m_N; i++){m_pV[i] = T(0);}}
template<class T>
const T& Vector<T>::operator () (size_t i) const
{
	return m_pV[i];
}
template<class T>
T& Vector<T>::operator () (size_t i)
{
	return m_pV[i];
}
template<class T>
const T& Vector<T>::operator [] (size_t i) const
{
	return m_pV[i];
}
template<class T>
T& Vector<T>::operator [] (size_t i)
{
	return m_pV[i];
}
template<class T>
Vector<T> Vector<T>::operator * (const T& A) const
{
	Vector V(*this);
	for (size_t i=0; i<m_N; i++) V.m_pV[i] *= A;
	return V;
}
template<class T>
Vector<T>& Vector<T>::operator *= (const T& A)
{
	for (size_t i=0; i<m_N; i++) m_pV[i] *= A;
	return *this;
}
template<class T>
Vector<T> Vector<T>::operator / (const T& A) const
{
	Vector V(*this);
	for (size_t i=0; i<m_N; i++) V.m_pV[i] /= A;
	return V;
}
template<class T>
Vector<T>& Vector<T>::operator /= (const T& A)
{
	for (size_t i=0; i<m_N; i++) m_pV[i] /= A;
	return *this;
}
template<class T>
Vector<T> Vector<T>::operator + (const Vector<T>& V0) const
{
	Vector<T> V(m_N);
	for (size_t i=0; i<m_N; i++) V.m_pV[i] = m_pV[i] + V0.m_pV[i];
	return V;
}
template<class T>
Vector<T>& Vector<T>::operator += (const Vector<T>& V)
{
	for ( size_t i=0 ; i<m_N ; i++ ) m_pV[i] += V.m_pV[i];
	return *this;
}
template<class T>
Vector<T> Vector<T>::operator - (const Vector<T>& V0) const
{
	Vector<T> V(m_N);
	for (size_t i=0; i<m_N; i++) V.m_pV[i] = m_pV[i] - V0.m_pV[i];
	return V;
}
template<class T>
Vector<T>& Vector<T>::operator -= (const Vector<T>& V)
{
	for (size_t i=0; i<m_N; i++) m_pV[i] -= V.m_pV[i];
	return *this;
}

template<class T>
Vector<T> Vector<T>::operator - (void) const
{
	Vector<T> V(m_N);
	for (size_t i=0; i<m_N; i++) V.m_pV[i] = -m_pV[i];
	return V;
}

template< class T >
Vector< T >& Vector< T >::Add( const Vector< T >* V , int count )
{
	for( int c=0 ; c<count ; c++ ) for( size_t i=0 ; i<m_N ; i++ ) m_pV[i] += V[c].m_pV[i];
	return *this;
}

template< class T >
Vector< T >& Vector< T >::AddScaled( const Vector<T>& V , const T& scale )
{
	for (size_t i=0; i<m_N; i++) m_pV[i] += V.m_pV[i]*scale;
	return *this;
}
template<class T>
Vector<T>& Vector<T>::SubtractScaled(const Vector<T>& V,const T& scale)
{
	for (size_t i=0; i<m_N; i++) m_pV[i] -= V.m_pV[i]*scale;
	return *this;
}

template<class T>
void Vector<T>::Add( const Vector<T>& V1 , const T& scale1 , const Vector<T>& V2 , const T& scale2 , Vector<T>& Out )
{
	for( size_t i=0 ; i<V1.m_N ; i++ ) Out.m_pV[i]=V1.m_pV[i]*scale1+V2.m_pV[i]*scale2;
}
template<class T>
void Vector<T>::Add(const Vector<T>& V1,const T& scale1,const Vector<T>& V2,Vector<T>& Out)
{
	for( size_t i=0 ; i<V1.m_N ; i++ ) Out.m_pV[i]=V1.m_pV[i]*scale1+V2.m_pV[i];
}

template<class T>
T Vector<T>::Norm( size_t Ln ) const
{
	T N = T();
	for (size_t i = 0; i<m_N; i++)
		N += pow(m_pV[i], (T)Ln);
	return pow(N, (T)1.0/Ln);	
}
template<class T>
void Vector<T>::Normalize()
{
	T N = 1.0f/Norm(2);
	for (size_t i = 0; i<m_N; i++)
		m_pV[i] *= N;
}
template<class T>
T Vector<T>::Length() const
{
	T N = T();
	for (size_t i = 0; i<m_N; i++)
		N += m_pV[i]*m_pV[i];
	return sqrt(N);	
}
template<class T>
T Vector<T>::Dot( const Vector<T>& V ) const
{
	T V0 = T();
	for( size_t i=0 ; i<m_N ; i++ ) V0 += m_pV[i]*V.m_pV[i];
	return V0;
}

template< class T >
bool Vector< T >::read( const char* fileName )
{
	FILE* fp = fopen( fileName , "rb" );
	if( !fp ) return false;
	bool ret = read( fp );
	fclose( fp );
	return ret;
}
template< class T >
bool Vector< T >::write( const char* fileName ) const
{
	FILE* fp = fopen( fileName , "wb" );
	if( !fp ) return false;
	bool ret = write( fp );
	fclose( fp );
	return ret;
}
template< class T >
bool Vector< T >::read( FILE* fp )
{
	int d;
	if( fread( &d , sizeof(int) , 1 , fp )!=1 ) return false;
	Resize( d );
	if( fread( &(*this)[0] , sizeof( T ) , d , fp )!=d ) return false;
	return true;
}
template< class T >
bool Vector< T >::write( FILE* fp ) const
{
	if( fwrite( &m_N , sizeof( int ) , 1 , fp )!=1 ) return false;
	if( fwrite( &(*this)[0] , sizeof( T ) , m_N , fp )!=m_N ) return false;
	return true;
}



#endif
