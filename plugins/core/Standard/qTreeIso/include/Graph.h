#pragma once
#include "Common.h"
#include <boost/graph/properties.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_traits.hpp>

namespace CP {

typedef boost::graph_traits<boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> >::edge_descriptor
    EdgeDescriptor;

template <typename T> class VertexAttribute
{
public:
    typedef T calc_type;
public:
    T weight; //weight of the observation
    std::vector<T> observation; //observed value
    std::vector<T> value; //current value
    uint32_t color; //field use for the graph cut
    bool isBorder; //is the node part of an activated edge
    uint32_t in_component; //index of the component in which the node belong
public:
    VertexAttribute(uint32_t dim = 1, T weight=1.)
        :weight(weight), observation(dim,0.),value(dim,0.),color(-1)
        ,isBorder(false){}
};

template <typename T> class EdgeAttribute
{
public:
    typedef T calc_type;
public:
    uint32_t index; //index of the edge (necessary for graph cuts)
    EdgeDescriptor edge_reverse; //pointer to the reverse edge, also necessary for graph cuts
    T weight; //weight of the edge
    T capacity; //capacity in the flow graph
    T residualCapacity; //necessary for graph cuts
    bool isActive; //is the edge in the support of the values
    bool realEdge; //is the edge between real nodes or link to source/sink
public:
    EdgeAttribute(T weight=1., uint32_t eIndex = 0, bool real = true)
        :index(eIndex), weight(weight), capacity(weight), residualCapacity(0), isActive(!real), realEdge(real) {}
};

template< typename T>
using Graph = typename boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexAttribute<T>, EdgeAttribute<T> >;

template< typename T>
using VertexDescriptor = typename boost::graph_traits<CP::Graph<T>>::vertex_descriptor;
template< typename T>
using VertexIndex    = typename boost::graph_traits<CP::Graph<T>>::vertices_size_type;
template< typename T>
using EdgeIndex     = typename boost::graph_traits<CP::Graph<T>>::edges_size_type;
template< typename T>
using VertexIterator = typename boost::graph_traits<Graph<T>>::vertex_iterator;
template< typename T>
using EdgeIterator   = typename boost::graph_traits<Graph<T>>::edge_iterator;

template<typename T>
using VertexAttributeMap = boost::vec_adj_list_vertex_property_map<Graph<T>, Graph<T>*
, VertexAttribute<T>, VertexAttribute<T> &,  boost::vertex_bundle_t >;
template<typename T>
using EdgeAttributeMap = boost::adj_list_edge_property_map<
boost::directed_tag, EdgeAttribute<T>, EdgeAttribute<T> &
, unsigned __int64, CP::EdgeAttribute<T>, boost::edge_bundle_t>;
template<typename T>
using VertexIndexMap = typename boost::property_map<Graph<T>, boost::vertex_index_t>::type;
template<typename T>
using EdgeIndexMap   = typename boost::property_map<Graph<T>, uint32_t EdgeAttribute<T>::*>::type;

template <typename T>
bool addDoubledge(Graph<T> & g, const VertexDescriptor<T> & source, const VertexDescriptor<T> & target
                  ,const T weight, uint32_t eIndex, EdgeAttributeMap<T> & edge_attribute_map, bool real = true)
{
       // Add edges between two vertices. We have to create the edge and the reverse edge,
       // then add the edge_reverse as the corresponding reverse edge to 'edge', and then add 'edge'
       // as the corresponding reverse edge to 'edge_reverse'

    	EdgeDescriptor edge, edge_reverse;
    	std::pair<EdgeDescriptor, bool> edge_added = boost::add_edge(source, target, g);
		if (edge_added.second)
		{
			edge = edge_added.first;
			edge_reverse      = boost::add_edge(target, source, g).first;
			EdgeAttribute<T> attrib_edge(weight,eIndex,real);
			EdgeAttribute<T> attrib_edge_reverse(weight,eIndex+1,real);
			attrib_edge.edge_reverse         = edge_reverse;
			attrib_edge_reverse.edge_reverse  = edge;
			edge_attribute_map(edge)        = attrib_edge;
			edge_attribute_map(edge_reverse) = attrib_edge_reverse;
			return true;
		}
		else
		{
			return false;
		}
}

}







