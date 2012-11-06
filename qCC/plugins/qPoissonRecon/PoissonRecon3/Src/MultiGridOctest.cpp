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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "Time.h"
#include "MarchingCubes.h"
#include "Octree.h"
#include "SparseMatrix.h"
#include "CmdLineParser.h"
#include "FunctionData.h"
#include "PPolynomial.h"
#include "ply.h"
#include "MemoryUsage.h"

#define SCALE 1.25

#include <stdarg.h>
char* outputFile=NULL;
int echoStdout=0;
void DumpOutput(const char* format,...){
	if(outputFile){
		FILE* fp=fopen(outputFile,"a");
		va_list args;
		va_start(args,format);
		vfprintf(fp,format,args);
		fclose(fp);
		va_end(args);
	}
	if(echoStdout){
		va_list args;
		va_start(args,format);
		vprintf(format,args);
		va_end(args);
	}
}
void DumpOutput2(char* str,const char* format,...){
	if(outputFile){
		FILE* fp=fopen(outputFile,"a");
		va_list args;
		va_start(args,format);
		vfprintf(fp,format,args);
		fclose(fp);
		va_end(args);
	}
	if(echoStdout){
		va_list args;
		va_start(args,format);
		vprintf(format,args);
		va_end(args);
	}
	va_list args;
	va_start(args,format);
	vsprintf(str,format,args);
	va_end(args);
	if(str[strlen(str)-1]=='\n'){str[strlen(str)-1]=0;}
}

#include "MultiGridOctreeData.h"


void ShowUsage(char* ex)
{
	printf("Usage: %s\n",ex);
	printf("\t--in  <input points>\n");

	printf("\t--out <ouput triangle mesh>\n");

	printf("\t[--depth <maximum reconstruction depth>]\n");
	printf("\t\t Running at depth d corresponds to solving on a 2^d x 2^d x 2^d\n");
	printf("\t\t voxel grid.\n");

	printf("\t[--scale <scale factor>]\n");
	printf("\t\t Specifies the factor of the bounding cube that the input\n");
	printf("\t\t samples should fit into.\n");

	printf("\t[--binary]\n");
	printf("\t\t If this flag is enabled, the point set is read in in\n");
	printf("\t\t binary format.\n");

	printf("\t[--solverDivide <subdivision depth>]\n");
	printf("\t\t The depth at which a block Gauss-Seidel solver is used\n");
	printf("\t\t to solve the Laplacian.\n");

	printf("\t[--samplesPerNode <minimum number of samples per node>\n");
	printf("\t\t This parameter specifies the minimum number of points that\n");
	printf("\t\t should fall within an octree node.\n");

	printf("\t[--confidence]\n");
	printf("\t\t If this flag is enabled, the size of a sample's normals is\n");
	printf("\t\t used as a confidence value, affecting the sample's\n");
	printf("\t\t constribution to the reconstruction process.\n");

	printf("\t[--manifold]\n");
	printf("\t\t If this flag is enabled, the isosurface extraction is performed\n");
	printf("\t\t by adding a polygon's barycenter in order to ensure that the output\n");
	printf("\t\t mesh is manifold.\n");

	printf("\t[--polygonMesh]\n");
	printf("\t\t If this flag is enabled, the isosurface extraction returns polygons\n");
	printf("\t\t rather than triangles. (This overrides the --manifold flag.)\n");

	printf("\t[--verbose]\n");
}
template<int Degree>
int Execute(int argc,char* argv[])
{
	int i;
	cmdLineString In,Out;
	cmdLineReadable Binary,Verbose,NoResetSamples,NoClipTree,Confidence,Manifold,PolygonMesh;
	cmdLineInt Depth(8),SolverDivide(8),IsoDivide(8),Refine(3);
	cmdLineInt KernelDepth;
	cmdLineFloat SamplesPerNode(1.0f),Scale(1.1f);
	char* paramNames[]=
	{
		"in","depth","out","refine","noResetSamples","noClipTree",
		"binary","solverDivide","isoDivide","scale","verbose",
		"kernelDepth","samplesPerNode","confidence","manifold","polygonMesh"
	};
	cmdLineReadable* params[]=
	{
		&In,&Depth,&Out,&Refine,&NoResetSamples,&NoClipTree,
		&Binary,&SolverDivide,&IsoDivide,&Scale,&Verbose,
		&KernelDepth,&SamplesPerNode,&Confidence,&Manifold,&PolygonMesh
	};
	int paramNum=sizeof(paramNames)/sizeof(char*);
	int commentNum=0;
	char **comments;

	comments=new char*[paramNum+7];
	for(i=0;i<paramNum+7;i++){comments[i]=new char[1024];}

	const char* Rev = "Rev: V2 ";
	const char* Date = "Date: 2006-11-09 (Thur, 09 Nov 2006) ";

	cmdLineParse(argc-1,&argv[1],paramNames,paramNum,params,0);

	if(Verbose.set){echoStdout=1;}

	DumpOutput2(comments[commentNum++],"Running Multi-Grid Octree Surface Reconstructor (degree %d). Version 3\n", Degree);
	if(In.set)				{DumpOutput2(comments[commentNum++],"\t--in %s\n",In.value);}
	if(Out.set)				{DumpOutput2(comments[commentNum++],"\t--out %s\n",Out.value);}
	if(Binary.set)			{DumpOutput2(comments[commentNum++],"\t--binary\n");}
	if(Depth.set)			{DumpOutput2(comments[commentNum++],"\t--depth %d\n",Depth.value);}
	if(SolverDivide.set)	{DumpOutput2(comments[commentNum++],"\t--solverDivide %d\n",SolverDivide.value);}
	if(IsoDivide.set)		{DumpOutput2(comments[commentNum++],"\t--isoDivide %d\n",IsoDivide.value);}
	if(Refine.set)			{DumpOutput2(comments[commentNum++],"\t--refine %d\n",Refine.value);}
	if(Scale.set)			{DumpOutput2(comments[commentNum++],"\t--scale %f\n",Scale.value);}
	if(KernelDepth.set)		{DumpOutput2(comments[commentNum++],"\t--kernelDepth %d\n",KernelDepth.value);}
	if(SamplesPerNode.set)	{DumpOutput2(comments[commentNum++],"\t--samplesPerNode %f\n",SamplesPerNode.value);}
	if(NoResetSamples.set)	{DumpOutput2(comments[commentNum++],"\t--noResetSamples\n");}
	if(NoClipTree.set)		{DumpOutput2(comments[commentNum++],"\t--noClipTree\n");}
	if(Confidence.set)		{DumpOutput2(comments[commentNum++],"\t--confidence\n");}
	if(Manifold.set)		{DumpOutput2(comments[commentNum++],"\t--manifold\n");}
	if(PolygonMesh.set)		{DumpOutput2(comments[commentNum++],"\t--polygonMesh\n");}

	double t;
	double tt=Time();
	Point3D<float> center;
	Real scale=1.0;
	Real isoValue=0;
	//////////////////////////////////
	// Fix courtesy of David Gallup //
	TreeNodeData::UseIndex = 1;     //
	//////////////////////////////////
	Octree<Degree> tree;
	PPolynomial<Degree> ReconstructionFunction=PPolynomial<Degree>::GaussianApproximation();

	center.coords[0]=center.coords[1]=center.coords[2]=0;
	if(!In.set || !Out.set)
	{
		ShowUsage(argv[0]);
		return 0;
	}
	
	TreeOctNode::SetAllocator(MEMORY_ALLOCATOR_BLOCK_SIZE);

	t=Time();
	int kernelDepth=Depth.value-2;
	if(KernelDepth.set){kernelDepth=KernelDepth.value;}

	tree.setFunctionData(ReconstructionFunction,Depth.value,0,Real(1.0)/(1<<Depth.value));
	DumpOutput("Function Data Set In: %lg\n",Time()-t);
	DumpOutput("Memory Usage: %.3f MB\n",float(MemoryInfo::Usage())/(1<<20));
	if(kernelDepth>Depth.value){
		fprintf(stderr,"KernelDepth can't be greater than Depth: %d <= %d\n",kernelDepth,Depth.value);
		return EXIT_FAILURE;
	}


	t=Time();
#if 1
	tree.setTree(In.value,Depth.value,Binary.set,kernelDepth,Real(SamplesPerNode.value),Scale.value,center,scale,!NoResetSamples.set,Confidence.set);
#else
if(Confidence.set){
	tree.setTree(In.value,Depth.value,Binary.set,kernelDepth,Real(SamplesPerNode.value),Scale.value,center,scale,!NoResetSamples.set,0,1);
}
else{
	tree.setTree(In.value,Depth.value,Binary.set,kernelDepth,Real(SamplesPerNode.value),Scale.value,center,scale,!NoResetSamples.set,0,0);
}
#endif
	DumpOutput2(comments[commentNum++],"#             Tree set in: %9.1f (s), %9.1f (MB)\n",Time()-t,tree.maxMemoryUsage);
	DumpOutput("Leaves/Nodes: %d/%d\n",tree.tree.leaves(),tree.tree.nodes());
	DumpOutput("   Tree Size: %.3f MB\n",float(sizeof(TreeOctNode)*tree.tree.nodes())/(1<<20));
	DumpOutput("Memory Usage: %.3f MB\n",float(MemoryInfo::Usage())/(1<<20));

	if(!NoClipTree.set){
		t=Time();
		tree.ClipTree();
		DumpOutput("Tree Clipped In: %lg\n",Time()-t);
		DumpOutput("Leaves/Nodes: %d/%d\n",tree.tree.leaves(),tree.tree.nodes());
		DumpOutput("   Tree Size: %.3f MB\n",float(sizeof(TreeOctNode)*tree.tree.nodes())/(1<<20));
	}

	t=Time();
	tree.finalize1(Refine.value);
	DumpOutput("Finalized 1 In: %lg\n",Time()-t);
	DumpOutput("Leaves/Nodes: %d/%d\n",tree.tree.leaves(),tree.tree.nodes());
	DumpOutput("Memory Usage: %.3f MB\n",float(MemoryInfo::Usage())/(1<<20));

	t=Time();
	tree.maxMemoryUsage=0;
	tree.SetLaplacianWeights();
	DumpOutput2(comments[commentNum++],"#Laplacian Weights Set In: %9.1f (s), %9.1f (MB)\n",Time()-t,tree.maxMemoryUsage);
	DumpOutput("Memory Usage: %.3f MB\n",float(MemoryInfo::Usage())/(1<<20));

	t=Time();
	tree.finalize2(Refine.value);
	DumpOutput("Finalized 2 In: %lg\n",Time()-t);
	DumpOutput("Leaves/Nodes: %d/%d\n",tree.tree.leaves(),tree.tree.nodes());
	DumpOutput("Memory Usage: %.3f MB\n",float(MemoryInfo::Usage())/(1<<20));

	tree.maxMemoryUsage=0;
	t=Time();
	tree.LaplacianMatrixIteration(SolverDivide.value);
	DumpOutput2(comments[commentNum++],"# Linear System Solved In: %9.1f (s), %9.1f (MB)\n",Time()-t,tree.maxMemoryUsage);
	DumpOutput("Memory Usage: %.3f MB\n",float(MemoryInfo::Usage())/(1<<20));

	CoredVectorMeshData mesh;
	tree.maxMemoryUsage=0;
	t=Time();
	isoValue=tree.GetIsoValue();
	DumpOutput("Got average in: %f\n",Time()-t);
	DumpOutput("Iso-Value: %e\n",isoValue);
	DumpOutput("Memory Usage: %.3f MB\n",float(tree.MemoryUsage()));

	t=Time();
	if(IsoDivide.value) tree.GetMCIsoTriangles( isoValue , IsoDivide.value , &mesh , 0 , 1 , Manifold.set , PolygonMesh.set );
	else                tree.GetMCIsoTriangles( isoValue ,                   &mesh , 0 , 1 , Manifold.set , PolygonMesh.set );
	if( PolygonMesh.set ) DumpOutput2(comments[commentNum++],"#         Got Polygons in: %9.1f (s), %9.1f (MB)\n",Time()-t,tree.maxMemoryUsage);
	else                  DumpOutput2(comments[commentNum++],"#        Got Triangles in: %9.1f (s), %9.1f (MB)\n",Time()-t,tree.maxMemoryUsage);
	DumpOutput2(comments[commentNum++],"#              Total Time: %9.1f (s)\n",Time()-tt);
	PlyWritePolygons(Out.value,&mesh,PLY_BINARY_NATIVE,center,scale,comments,commentNum);

	return 1;
}

int main(int argc,char* argv[])
{
	int degree=2;

	switch(degree)
	{
		case 1:
			Execute<1>(argc,argv);
			break;
		case 2:
			Execute<2>(argc,argv);
			break;
		case 3:
			Execute<3>(argc,argv);
			break;
		case 4:
			Execute<4>(argc,argv);
			break;
		case 5:
			Execute<5>(argc,argv);
			break;
		default:
			fprintf(stderr,"Degree %d not supported\n",degree);
			return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
