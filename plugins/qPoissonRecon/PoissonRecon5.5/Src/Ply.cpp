#include "Ply.h"

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

template<>
PlyProperty PlyValueVertex< float >::Properties[]=
{
   {"x", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyValueVertex,point.coords[0])), 0, 0, 0, 0},
   {"y", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyValueVertex,point.coords[1])), 0, 0, 0, 0},
   {"z", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyValueVertex,point.coords[2])), 0, 0, 0, 0},
   {"value", PLY_FLOAT, PLY_FLOAT, int(offsetof(PlyValueVertex,value)), 0, 0, 0, 0}
};
template<>
PlyProperty PlyValueVertex< double >::Properties[]=
{
   {"x", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyValueVertex,point.coords[0])), 0, 0, 0, 0},
   {"y", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyValueVertex,point.coords[1])), 0, 0, 0, 0},
   {"z", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyValueVertex,point.coords[2])), 0, 0, 0, 0},
   {"value", PLY_DOUBLE, PLY_DOUBLE, int(offsetof(PlyValueVertex,value)), 0, 0, 0, 0}
};

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
