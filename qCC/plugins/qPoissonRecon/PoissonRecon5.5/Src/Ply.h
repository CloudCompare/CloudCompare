/*

 Header for PLY polygon files.
 
  - Greg Turk, March 1994
  
   A PLY file contains a single polygonal _object_.
   
	An object is composed of lists of _elements_.  Typical elements are
	vertices, faces, edges and materials.
	
	 Each type of element for a given object has one or more _properties_
	 associated with the element type.  For instance, a vertex element may
	 have as properties three floating-point values x,y,z and three unsigned
	 chars for red, green and blue.
	 
	  ---------------------------------------------------------------
	  
	   Copyright (c) 1994 The Board of Trustees of The Leland Stanford
	   Junior University.  All rights reserved.   
	   
		Permission to use, copy, modify and distribute this software and its   
		documentation for any purpose is hereby granted without fee, provided   
		that the above copyright notice and this permission notice appear in   
		all copies of this software and that you do not sell the software.   
		
		 THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND,   
		 EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY   
		 WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.   
		 
*/

#ifndef __PLY_H__
#define __PLY_H__

#ifndef WIN32
#define _strdup strdup
#endif

#ifdef __cplusplus
extern "C" {
#endif
	
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>

#define PLY_ASCII         1      /* ascii PLY file */
#define PLY_BINARY_BE     2      /* binary PLY file, big endian */
#define PLY_BINARY_LE     3      /* binary PLY file, little endian */
#define PLY_BINARY_NATIVE 4      /* binary PLY file, same endianness as current architecture */
    
#define PLY_OKAY    0           /* ply routine worked okay */
#define PLY_ERROR  -1           /* error in ply routine */
	
	/* scalar data types supported by PLY format */
	
#define PLY_START_TYPE 0
#define PLY_CHAR       1
#define PLY_SHORT      2
#define PLY_INT        3
#define PLY_UCHAR      4
#define PLY_USHORT     5
#define PLY_UINT       6
#define PLY_FLOAT      7
#define PLY_DOUBLE     8
#define PLY_INT_8      9
#define PLY_UINT_8     10
#define PLY_INT_16     11
#define PLY_UINT_16    12
#define PLY_INT_32     13
#define PLY_UINT_32    14
#define PLY_FLOAT_32   15
#define PLY_FLOAT_64   16
	
#define PLY_END_TYPE   17
	
#define  PLY_SCALAR  0
#define  PLY_LIST    1
	

typedef struct PlyElement {     /* description of an element */
	char *name;                   /* element name */
	int num;                      /* number of elements in this object */
	int size;                     /* size of element (bytes) or -1 if variable */
	int nprops;                   /* number of properties for this element */
	PlyProperty **props;          /* list of properties in the file */
	char *store_prop;             /* flags: property wanted by user? */
	int other_offset;             /* offset to un-asked-for props, or -1 if none*/
	int other_size;               /* size of other_props structure */
} PlyElement;

typedef struct PlyOtherProp {   /* describes other properties in an element */
	char *name;                   /* element name */
	int size;                     /* size of other_props */
	int nprops;                   /* number of properties in other_props */
	PlyProperty **props;          /* list of properties in other_props */
} PlyOtherProp;

typedef struct OtherData { /* for storing other_props for an other element */
	void *other_props;
} OtherData;

typedef struct OtherElem {     /* data for one "other" element */
	char *elem_name;             /* names of other elements */
	int elem_count;              /* count of instances of each element */
	OtherData **other_data;      /* actual property data for the elements */
	PlyOtherProp *other_props;   /* description of the property data */
} OtherElem;

typedef struct PlyOtherElems {  /* "other" elements, not interpreted by user */
	int num_elems;                /* number of other elements */
	OtherElem *other_list;        /* list of data for other elements */
} PlyOtherElems;

typedef struct PlyFile {        /* description of PLY file */
	FILE *fp;                     /* file pointer */
	int file_type;                /* ascii or binary */
	float version;                /* version number of file */
	int nelems;                   /* number of elements of object */
	PlyElement **elems;           /* list of elements */
	int num_comments;             /* number of comments */
	char **comments;              /* list of comments */
	int num_obj_info;             /* number of items of object information */
	char **obj_info;              /* list of object info items */
	PlyElement *which_elem;       /* which element we're currently writing */
	PlyOtherElems *other_elems;   /* "other" elements from a PLY file */
} PlyFile;
	
	/* memory allocation */
extern char *my_alloc();
#define myalloc(mem_size) my_alloc((mem_size), __LINE__, __FILE__)

#ifndef ALLOCN
#define REALLOCN(PTR,TYPE,OLD_N,NEW_N)							\
{										\
	if ((OLD_N) == 0)                                           		\
{   ALLOCN((PTR),TYPE,(NEW_N));}                            		\
	else									\
{								    		\
	(PTR) = (TYPE *)realloc((PTR),(NEW_N)*sizeof(TYPE));			\
	if (((PTR) == NULL) && ((NEW_N) != 0))					\
{									\
	fprintf(stderr, "Memory reallocation failed on line %d in %s\n", 	\
	__LINE__, __FILE__);                             		\
	fprintf(stderr, "  tried to reallocate %d->%d\n",       		\
	(OLD_N), (NEW_N));                              		\
	exit(-1);								\
}									\
	if ((NEW_N)>(OLD_N))							\
	memset((char *)(PTR)+(OLD_N)*sizeof(TYPE), 0,			\
	((NEW_N)-(OLD_N))*sizeof(TYPE));				\
}										\
}

#define  ALLOCN(PTR,TYPE,N) 					\
{ (PTR) = (TYPE *) calloc(((unsigned)(N)),sizeof(TYPE));\
	if ((PTR) == NULL) {    				\
	fprintf(stderr, "Memory allocation failed on line %d in %s\n", \
	__LINE__, __FILE__);                           \
	exit(-1);                                             \
	}							\
}


#define FREE(PTR)  { free((PTR)); (PTR) = NULL; }
#endif


/*** delcaration of routines ***/

extern PlyFile *ply_write(FILE *, int, char **, int);
extern PlyFile *ply_open_for_writing(char *, int, char **, int, float *);
extern void ply_describe_element(PlyFile *, char *, int, int, PlyProperty *);
extern void ply_describe_property(PlyFile *, char *, PlyProperty *);
extern void ply_element_count(PlyFile *, char *, int);
extern void ply_header_complete(PlyFile *);
extern void ply_put_element_setup(PlyFile *, char *);
extern void ply_put_element(PlyFile *, void *);
extern void ply_put_comment(PlyFile *, char *);
extern void ply_put_obj_info(PlyFile *, char *);
extern PlyFile *ply_read(FILE *, int *, char ***);
extern PlyFile *ply_open_for_reading( char *, int *, char ***, int *, float *);
extern PlyProperty **ply_get_element_description(PlyFile *, char *, int*, int*);
extern void ply_get_element_setup( PlyFile *, char *, int, PlyProperty *);
extern int ply_get_property(PlyFile *, char *, PlyProperty *);
extern PlyOtherProp *ply_get_other_properties(PlyFile *, char *, int);
extern void ply_get_element(PlyFile *, void *);
extern char **ply_get_comments(PlyFile *, int *);
extern char **ply_get_obj_info(PlyFile *, int *);
extern void ply_close(PlyFile *);
extern void ply_get_info(PlyFile *, float *, int *);
extern PlyOtherElems *ply_get_other_element (PlyFile *, char *, int);
extern void ply_describe_other_elements ( PlyFile *, PlyOtherElems *);
extern void ply_put_other_elements (PlyFile *);
extern void ply_free_other_elements (PlyOtherElems *);
extern void ply_describe_other_properties(PlyFile *, PlyOtherProp *, int);

extern int equal_strings(char *, char *);

#ifdef __cplusplus
}
#endif
#include "Geometry.h"
#include <vector>
#include "PlyVertex.h"

typedef struct PlyFace
{
	unsigned char nr_vertices;
	int *vertices;
	int segment;
} PlyFace;
static PlyProperty face_props[] =
{
	{ "vertex_indices" , PLY_INT , PLY_INT , offsetof(PlyFace,vertices) , 1 , PLY_UCHAR, PLY_UCHAR , offsetof(PlyFace,nr_vertices) },
};

template< class Real > PlyVertex< Real > operator * ( XForm4x4< Real > xForm , PlyVertex< Real > v ) { return PlyVertex< Real >( xForm * v.point ); }
template<>
PlyProperty PlyVertex< float >::Properties[]=
{
	{"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyVertex,point.coords[0])), 0, 0, 0, 0},
	{"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyVertex,point.coords[1])), 0, 0, 0, 0},
	{"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyVertex,point.coords[2])), 0, 0, 0, 0}
};
template<>
PlyProperty PlyVertex< double >::Properties[]=
{
	{"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyVertex,point.coords[0])), 0, 0, 0, 0},
	{"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyVertex,point.coords[1])), 0, 0, 0, 0},
	{"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyVertex,point.coords[2])), 0, 0, 0, 0}
};
template< class Real >
class PlyValueVertex
{
public:
	const static int Components=4;
	static PlyProperty Properties[];

	Point3D<Real> point;
	Real value;

#if 1
	PlyValueVertex( void ) : value( Real(0) ) { ; }
	PlyValueVertex( Point3D< Real > p , Real v ) : point(p) , value(v) { ; }
	PlyValueVertex operator + ( PlyValueVertex p ) const { return PlyValueVertex( point+p.point , value+p.value ); }
	PlyValueVertex operator - ( PlyValueVertex p ) const { return PlyValueVertex( point-p.value , value-p.value ); }
	PlyValueVertex operator * ( Real s ) const { return PlyValueVertex( point*s , value*s ); }
	PlyValueVertex operator / ( Real s ) const { return PlyValueVertex( point/s , value/s ); }
	PlyValueVertex& operator += ( PlyValueVertex p ) { point += p.point , value += p.value ; return *this; }
	PlyValueVertex& operator -= ( PlyValueVertex p ) { point -= p.point , value -= p.value ; return *this; }
	PlyValueVertex& operator *= ( Real s ) { point *= s , value *= s ; return *this; }
	PlyValueVertex& operator /= ( Real s ) { point /= s , value /= s ; return *this; }
#else
	operator Point3D<Real>& ()				{return point;}
	operator const Point3D<Real>& () const	{return point;}
	PlyValueVertex(void)					{point.coords[0]=point.coords[1]=point.coords[2]=0;value=0;}
	PlyValueVertex(const Point3D<Real>& p)	{point=p;}
#endif
};
template< class Real > PlyValueVertex< Real > operator * ( XForm4x4< Real > xForm , PlyValueVertex< Real > v ) { return PlyValueVertex< Real >( xForm * v.point , v.value ); }
template< >
PlyProperty PlyValueVertex< float >::Properties[]=
{
	{"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyValueVertex,point.coords[0])), 0, 0, 0, 0},
	{"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyValueVertex,point.coords[1])), 0, 0, 0, 0},
	{"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyValueVertex,point.coords[2])), 0, 0, 0, 0},
	{"value", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyValueVertex,value)), 0, 0, 0, 0}
};
template< >
PlyProperty PlyValueVertex< double >::Properties[]=
{
	{"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyValueVertex,point.coords[0])), 0, 0, 0, 0},
	{"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyValueVertex,point.coords[1])), 0, 0, 0, 0},
	{"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyValueVertex,point.coords[2])), 0, 0, 0, 0},
	{"value", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyValueVertex,value)), 0, 0, 0, 0}
};
template< class Real >
class PlyOrientedVertex
{
public:
	const static int Components=6;
	static PlyProperty Properties[];

	Point3D<Real> point , normal;

#if 1
	PlyOrientedVertex( void ) { ; }
	PlyOrientedVertex( Point3D< Real > p , Point3D< Real > n ) : point(p) , normal(n) { ; }
  	PlyOrientedVertex operator + ( PlyOrientedVertex p ) const { return PlyOrientedVertex( point+p.point , normal+p.normal ); }
	PlyOrientedVertex operator - ( PlyOrientedVertex p ) const { return PlyOrientedVertex( point-p.value , normal-p.normal ); }
	PlyOrientedVertex operator * ( Real s ) const { return PlyOrientedVertex( point*s , normal*s ); }
	PlyOrientedVertex operator / ( Real s ) const { return PlyOrientedVertex( point/s , normal/s ); }
	PlyOrientedVertex& operator += ( PlyOrientedVertex p ) { point += p.point , normal += p.normal ; return *this; }
	PlyOrientedVertex& operator -= ( PlyOrientedVertex p ) { point -= p.point , normal -= p.normal ; return *this; }
	PlyOrientedVertex& operator *= ( Real s ) { point *= s , normal *= s ; return *this; }
	PlyOrientedVertex& operator /= ( Real s ) { point /= s , normal /= s ; return *this; }
#else
	operator Point3D<Real>& ()					{return point;}
	operator const Point3D<Real>& () const		{return point;}
	PlyOrientedVertex(void)						{point.coords[0]=point.coords[1]=point.coords[2]=normal.coords[0]=normal.coords[1]=normal.coords[2]=0;}
	PlyOrientedVertex(const Point3D<Real>& p)	{point=p;}
#endif
};
template< class Real > PlyOrientedVertex< Real > operator * ( XForm4x4< Real > xForm , PlyOrientedVertex< Real > v ) { return PlyOrientedVertex< Real >( xForm * v.point , xForm.inverse().transpose() * v.normal ); }
template<>
PlyProperty PlyOrientedVertex< float >::Properties[]=
{
	{"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,point.coords[0])), 0, 0, 0, 0},
	{"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,point.coords[1])), 0, 0, 0, 0},
	{"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,point.coords[2])), 0, 0, 0, 0},
	{"nx", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,normal.coords[0])), 0, 0, 0, 0},
	{"ny", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,normal.coords[1])), 0, 0, 0, 0},
	{"nz", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyOrientedVertex,normal.coords[2])), 0, 0, 0, 0}
};
template<>
PlyProperty PlyOrientedVertex< double >::Properties[]=
{
	{"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,point.coords[0])), 0, 0, 0, 0},
	{"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,point.coords[1])), 0, 0, 0, 0},
	{"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,point.coords[2])), 0, 0, 0, 0},
	{"nx", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,normal.coords[0])), 0, 0, 0, 0},
	{"ny", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,normal.coords[1])), 0, 0, 0, 0},
	{"nz", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyOrientedVertex,normal.coords[2])), 0, 0, 0, 0}
};
template< class Real >
class PlyColorVertex
{
public:
	const static int Components=6;
	static PlyProperty Properties[];

	Point3D<Real> point;
	unsigned char color[3];

	operator Point3D<Real>& ()					{return point;}
	operator const Point3D<Real>& () const		{return point;}
	PlyColorVertex(void)						{point.coords[0]=point.coords[1]=point.coords[2]=0,color[0]=color[1]=color[2]=0;}
	PlyColorVertex(const Point3D<Real>& p)	{point=p;}
};
template<>
PlyProperty PlyColorVertex< float >::Properties[]=
{
	{"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyColorVertex,point.coords[0])), 0, 0, 0, 0},
	{"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyColorVertex,point.coords[1])), 0, 0, 0, 0},
	{"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyColorVertex,point.coords[2])), 0, 0, 0, 0},
	{"red",		PLY_UCHAR, PLY_UCHAR, int(offsetof(PlyColorVertex,color[0])),	0, 0, 0, 0},
	{"green",	PLY_UCHAR, PLY_UCHAR, int(offsetof(PlyColorVertex,color[1])),	0, 0, 0, 0},
	{"blue",	PLY_UCHAR, PLY_UCHAR, int(offsetof(PlyColorVertex,color[2])),	0, 0, 0, 0}
};
template<>
PlyProperty PlyColorVertex< double >::Properties[]=
{
	{"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyColorVertex,point.coords[0])), 0, 0, 0, 0},
	{"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyColorVertex,point.coords[1])), 0, 0, 0, 0},
	{"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyColorVertex,point.coords[2])), 0, 0, 0, 0},
	{"red",		PLY_UCHAR, PLY_UCHAR, int(offsetof(PlyColorVertex,color[0])),	0, 0, 0, 0},
	{"green",	PLY_UCHAR, PLY_UCHAR, int(offsetof(PlyColorVertex,color[1])),	0, 0, 0, 0},
	{"blue",	PLY_UCHAR, PLY_UCHAR, int(offsetof(PlyColorVertex,color[2])),	0, 0, 0, 0}
};

template< class Vertex >
int PlyWritePolygons( char* fileName , CoredMeshData< Vertex >*  mesh , int file_type , const Point3D< float >& translate , float scale , char** comments=NULL , int commentNum=0 , XForm4x4< float > xForm=XForm4x4< float >::Identity() );

template< class Vertex >
int PlyWritePolygons( char* fileName , CoredMeshData< Vertex >*  mesh , int file_type , char** comments=NULL , int commentNum=0 , XForm4x4< float > xForm=XForm4x4< float >::Identity() );

template<class Vertex>
int PlyReadPolygons(char* fileName,
					std::vector<Vertex>& vertices,std::vector<std::vector<int> >& polygons,
					PlyProperty* properties,int propertyNum,
					int& file_type,
					char*** comments=NULL,int* commentNum=NULL , bool* readFlags=NULL );

template<class Vertex>
int PlyWritePolygons(char* fileName,
					 const std::vector<Vertex>& vertices,const std::vector<std::vector<int> >& polygons,
					 PlyProperty* properties,int propertyNum,
					 int file_type,
					 char** comments=NULL,const int& commentNum=0);

template<class Vertex>
int PlyWritePolygons(char* fileName,
					 const std::vector<Vertex>& vertices , const std::vector< std::vector< int > >& polygons,
					 PlyProperty* properties,int propertyNum,
					 int file_type,
					 char** comments,const int& commentNum)
{
	int nr_vertices=int(vertices.size());
	int nr_faces=int(polygons.size());
	float version;
	char *elem_names[] = { "vertex" , "face" };
	PlyFile *ply = ply_open_for_writing( fileName , 2 , elem_names , file_type , &version );
	if (!ply){return 0;}
	
	//
	// describe vertex and face properties
	//
	ply_element_count(ply, "vertex", nr_vertices);
	for(int i=0;i<propertyNum;i++)
		ply_describe_property(ply, "vertex", &properties[i]);
	
	ply_element_count(ply, "face", nr_faces);
	ply_describe_property(ply, "face", &face_props[0]);
	
	// Write in the comments
	if(comments && commentNum)
		for(int i=0;i<commentNum;i++)
			ply_put_comment(ply,comments[i]);

	ply_header_complete(ply);
	
	// write vertices
	ply_put_element_setup(ply, "vertex");
	for (int i=0; i < int(vertices.size()); i++)
		ply_put_element(ply, (void *) &vertices[i]);

	// write faces
	PlyFace ply_face;
	int maxFaceVerts=3;
	ply_face.nr_vertices = 3;
	ply_face.vertices = new int[3];

	ply_put_element_setup(ply, "face");
	for (int i=0; i < nr_faces; i++)
	{
		if(int(polygons[i].size())>maxFaceVerts)
		{
			delete[] ply_face.vertices;
			maxFaceVerts=int(polygons[i].size());
			ply_face.vertices=new int[maxFaceVerts];
		}
		ply_face.nr_vertices=int(polygons[i].size());
		for(int j=0;j<ply_face.nr_vertices;j++)
			ply_face.vertices[j]=polygons[i][j];
		ply_put_element(ply, (void *) &ply_face);
	}

	delete[] ply_face.vertices;
	ply_close(ply);
	return 1;
}
template<class Vertex>
int PlyReadPolygons(char* fileName,
					std::vector<Vertex>& vertices , std::vector<std::vector<int> >& polygons ,
					 PlyProperty* properties , int propertyNum ,
					int& file_type ,
					char*** comments , int* commentNum , bool* readFlags )
{
	int nr_elems;
	char **elist;
	float version;
	int i,j,k;
	PlyFile* ply;
	char* elem_name;
	int num_elems;
	int nr_props;
	PlyProperty** plist;
	PlyFace ply_face;

	ply = ply_open_for_reading(fileName, &nr_elems, &elist, &file_type, &version);
	if(!ply) return 0;

	if(comments)
	{
		(*comments)=new char*[*commentNum+ply->num_comments];
		for(int i=0;i<ply->num_comments;i++)
			(*comments)[i]=_strdup(ply->comments[i]);
		*commentNum=ply->num_comments;
	}

	for (i=0; i < nr_elems; i++) {
		elem_name = elist[i];
		plist = ply_get_element_description(ply, elem_name, &num_elems, &nr_props);
		if(!plist)
		{
			for(i=0;i<nr_elems;i++){
				free(ply->elems[i]->name);
				free(ply->elems[i]->store_prop);
				for(j=0;j<ply->elems[i]->nprops;j++){
					free(ply->elems[i]->props[j]->name);
					free(ply->elems[i]->props[j]);
				}
				free(ply->elems[i]->props);
			}
			for(i=0;i<nr_elems;i++){free(ply->elems[i]);}
			free(ply->elems);
			for(i=0;i<ply->num_comments;i++){free(ply->comments[i]);}
			free(ply->comments);
			for(i=0;i<ply->num_obj_info;i++){free(ply->obj_info[i]);}
			free(ply->obj_info);
			ply_free_other_elements (ply->other_elems);
			
			for(i=0;i<nr_elems;i++){free(elist[i]);}
			free(elist);
			ply_close(ply);
			return 0;
		}		
		if (equal_strings("vertex", elem_name))
		{
			for( int i=0 ; i<propertyNum ; i++)
			{
				int hasProperty = ply_get_property(ply,elem_name,&properties[i]);
				if( readFlags ) readFlags[i] = (hasProperty!=0);
			}
			vertices.resize(num_elems);
			for (j=0; j < num_elems; j++)	ply_get_element (ply, (void *) &vertices[j]);
		}
		else if (equal_strings("face", elem_name))
		{
			ply_get_property (ply, elem_name, &face_props[0]);
			polygons.resize(num_elems);
			for (j=0; j < num_elems; j++)
			{
				ply_get_element (ply, (void *) &ply_face);
				polygons[j].resize(ply_face.nr_vertices);
				for(k=0;k<ply_face.nr_vertices;k++)	polygons[j][k]=ply_face.vertices[k];
				delete[] ply_face.vertices;
			}  // for, read faces
		}  // if face
		else{ply_get_other_element (ply, elem_name, num_elems);}

		for(j=0;j<nr_props;j++){
			free(plist[j]->name);
			free(plist[j]);
		}
		free(plist);
	}  // for each type of element
	
	for(i=0;i<nr_elems;i++){
		free(ply->elems[i]->name);
		free(ply->elems[i]->store_prop);
		for(j=0;j<ply->elems[i]->nprops;j++){
			free(ply->elems[i]->props[j]->name);
			free(ply->elems[i]->props[j]);
		}
		if(ply->elems[i]->props && ply->elems[i]->nprops){free(ply->elems[i]->props);}
	}
	for(i=0;i<nr_elems;i++){free(ply->elems[i]);}
	free(ply->elems);
	for(i=0;i<ply->num_comments;i++){free(ply->comments[i]);}
	free(ply->comments);
	for(i=0;i<ply->num_obj_info;i++){free(ply->obj_info[i]);}
	free(ply->obj_info);
	ply_free_other_elements (ply->other_elems);
	
	
	for(i=0;i<nr_elems;i++){free(elist[i]);}
	free(elist);
	ply_close(ply);
	return 1;
}

template< class Vertex >
int PlyWritePolygons( char* fileName , CoredMeshData< Vertex >* mesh , int file_type , const Point3D<float>& translate , float scale , char** comments , int commentNum , XForm4x4< float > xForm )
{
	XForm3x3< float > xFormN;
	for( int i=0 ; i<3 ; i++ ) for( int j=0 ; j<3 ; j++ ) xFormN(i,j) = xForm(i,j);
	xFormN = xFormN.transpose().inverse();
	int i;
	int nr_vertices=int(mesh->outOfCorePointCount()+mesh->inCorePoints.size());
	int nr_faces=mesh->polygonCount();
	float version;
	char *elem_names[] = { "vertex" , "face" };
	PlyFile *ply = ply_open_for_writing( fileName , 2 , elem_names , file_type , &version );
	if( !ply ) return 0;

	mesh->resetIterator();
	
	//
	// describe vertex and face properties
	//
	ply_element_count( ply , "vertex" , nr_vertices );
	for( int i=0 ; i<Vertex::Components ; i++ ) ply_describe_property( ply , "vertex" , &Vertex::Properties[i] );
	
	ply_element_count( ply , "face" , nr_faces );
	ply_describe_property( ply , "face" , &face_props[0] );
	
	// Write in the comments
	for( i=0 ; i<commentNum ; i++ ) ply_put_comment( ply , comments[i] );

	ply_header_complete( ply );
	
	// write vertices
	ply_put_element_setup( ply , "vertex" );
	Point3D< float > p;
	for( i=0 ; i<int( mesh->inCorePoints.size() ) ; i++ )
	{
		Vertex vertex = xForm * ( mesh->inCorePoints[i] * scale + translate );
		ply_put_element(ply, (void *) &vertex);
	}
	for( i=0; i<mesh->outOfCorePointCount() ; i++ )
	{
		Vertex vertex;
		mesh->nextOutOfCorePoint( vertex );
		vertex = xForm * ( vertex * scale +translate );
		ply_put_element(ply, (void *) &vertex);		
	}  // for, write vertices
	
	// write faces
	std::vector< CoredVertexIndex > polygon;
	ply_put_element_setup( ply , "face" );
	for( i=0 ; i<nr_faces ; i++ )
	{
		//
		// create and fill a struct that the ply code can handle
		//
		PlyFace ply_face;
		mesh->nextPolygon( polygon );
		ply_face.nr_vertices = int( polygon.size() );
		ply_face.vertices = new int[ polygon.size() ];
		for( int i=0 ; i<int(polygon.size()) ; i++ )
			if( polygon[i].inCore ) ply_face.vertices[i] = polygon[i].idx;
			else                    ply_face.vertices[i] = polygon[i].idx + int( mesh->inCorePoints.size() );
		ply_put_element( ply, (void *) &ply_face );
		delete[] ply_face.vertices;
	}  // for, write faces
	
	ply_close( ply );
	return 1;
}
template< class Vertex >
int PlyWritePolygons( char* fileName , CoredMeshData< Vertex >* mesh , int file_type , char** comments , int commentNum , XForm4x4< float > xForm )
{
	XForm3x3< float > xFormN;
	for( int i=0 ; i<3 ; i++ ) for( int j=0 ; j<3 ; j++ ) xFormN(i,j) = xForm(i,j);
	xFormN = xFormN.transpose().inverse();
	int i;
	int nr_vertices=int(mesh->outOfCorePointCount()+mesh->inCorePoints.size());
	int nr_faces=mesh->polygonCount();
	float version;
	char *elem_names[] = { "vertex" , "face" };
	PlyFile *ply = ply_open_for_writing( fileName , 2 , elem_names , file_type , &version );
	if( !ply ) return 0;

	mesh->resetIterator();
	
	//
	// describe vertex and face properties
	//
	ply_element_count( ply , "vertex" , nr_vertices );
	for( int i=0 ; i<Vertex::Components ; i++ ) ply_describe_property( ply , "vertex" , &Vertex::Properties[i] );
	
	ply_element_count( ply , "face" , nr_faces );
	ply_describe_property( ply , "face" , &face_props[0] );
	
	// Write in the comments
	for( i=0 ; i<commentNum ; i++ ) ply_put_comment( ply , comments[i] );

	ply_header_complete( ply );
	
	// write vertices
	ply_put_element_setup( ply , "vertex" );
	Point3D< float > p;
	for( i=0 ; i<int( mesh->inCorePoints.size() ) ; i++ )
	{
		Vertex vertex = xForm * mesh->inCorePoints[i];
		ply_put_element(ply, (void *) &vertex);
	}
	for( i=0; i<mesh->outOfCorePointCount() ; i++ )
	{
		Vertex vertex;
		mesh->nextOutOfCorePoint( vertex );
		vertex = xForm * ( vertex );
		ply_put_element(ply, (void *) &vertex);		
	}  // for, write vertices
	
	// write faces
	std::vector< CoredVertexIndex > polygon;
	ply_put_element_setup( ply , "face" );
	for( i=0 ; i<nr_faces ; i++ )
	{
		//
		// create and fill a struct that the ply code can handle
		//
		PlyFace ply_face;
		mesh->nextPolygon( polygon );
		ply_face.nr_vertices = int( polygon.size() );
		ply_face.vertices = new int[ polygon.size() ];
		for( int i=0 ; i<int(polygon.size()) ; i++ )
			if( polygon[i].inCore ) ply_face.vertices[i] = polygon[i].idx;
			else                    ply_face.vertices[i] = polygon[i].idx + int( mesh->inCorePoints.size() );
		ply_put_element( ply, (void *) &ply_face );
		delete[] ply_face.vertices;
	}  // for, write faces
	
	ply_close( ply );
	return 1;
}
inline int PlyDefaultFileType(void){return PLY_ASCII;}

#endif /* !__PLY_H__ */
