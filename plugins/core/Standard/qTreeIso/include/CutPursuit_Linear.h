#pragma once
#include "Common.h"
#include "CutPursuit.h"

namespace CP {

template <typename T>
class CutPursuit_Linear : public CutPursuit<T>
{
    public:
   ~CutPursuit_Linear(){
    };        
   // virtual ~CutPursuit_Linear();  
    std::vector<std::vector<T>> componentVector;
        // only used with backward step - the sum of all observation in the component
    CutPursuit_Linear(uint32_t nbVertex = 1) : CutPursuit<T>(nbVertex)
    {
        this->componentVector  = std::vector<std::vector<T>>(1);
    }

    virtual std::pair<T,T> compute_energy() override
    {
        VertexAttributeMap<T> vertex_attribute_map
                = boost::get(boost::vertex_bundle, this->main_graph);
        EdgeAttributeMap<T> edge_attribute_map
                = boost::get(boost::edge_bundle, this->main_graph);
        std::pair<T,T> pair_energy;
        T energy = 0;
        VertexIterator<T> i_ver;
        //#pragma omp parallel for private(i_ver) if (this->parameter.parallel)
        for (i_ver = boost::vertices(this->main_graph).first;
             i_ver != this->lastIterator; ++i_ver)
        {
            for(uint32_t i_dim=0; i_dim<this->dim; i_dim++)
            {
                energy -= vertex_attribute_map(*i_ver).weight
                        * vertex_attribute_map(*i_ver).observation[i_dim]
                        * vertex_attribute_map(*i_ver).value[i_dim];
            }
        }
        pair_energy.first = energy;
        energy = 0;
        EdgeIterator<T> i_edg, i_edg_end =  boost::edges(this->main_graph).second;
        for (i_edg = boost::edges(this->main_graph).first;
             i_edg != i_edg_end; ++i_edg)
        {
            if (!edge_attribute_map(*i_edg).realEdge)
            {
                continue;
            }
            energy += .5 * edge_attribute_map(*i_edg).isActive * this->parameter.reg_strenth
                    * edge_attribute_map(*i_edg).weight;
        }
        pair_energy.second = energy;
        return pair_energy;
    }

    //=============================================================================================
    //=============================        SPLIT        ===========================================
    //=============================================================================================
    virtual uint32_t split()
    { // split the graph by trying to find the best binary partition
      // each components is split into B and notB
        uint32_t saturation;
        //initialize h_1 and h_2 with kmeans
        //--------initilializing labels------------------------------------------------------------
        //corner contains the two most likely class for each component
        std::vector< std::vector< uint32_t > > corners =
                std::vector< std::vector< uint32_t > >(this->components.size(),
                std::vector< uint32_t >(2,0));
        this->compute_corners(corners);
        this->set_capacities(corners);
        //compute flow
        boost::boykov_kolmogorov_max_flow(
                   this->main_graph,
                   get(&EdgeAttribute<T>::capacity        , this->main_graph),
                   get(&EdgeAttribute<T>::residualCapacity, this->main_graph),
                   get(&EdgeAttribute<T>::edge_reverse     , this->main_graph),
                   get(&VertexAttribute<T>::color         , this->main_graph),
                   get(boost::vertex_index                , this->main_graph),
                   this->source,
                   this->sink);
        saturation = this->activate_edges();
        return saturation;
    }
    //=============================================================================================
    //=============================      COMPUTE CORNERS        ===================================
    //=============================================================================================
    inline void compute_corners(std::vector< std::vector< uint32_t > > & corners)
    { //-----compute the 2 most populous labels------------------------------
         //#pragma omp parallel for if (this->parameter.parallel) schedule(dynamic)
        for (uint32_t  i_com =0;i_com < this->components.size(); i_com++)
        {
            if (this->saturated_components[i_com])
            {
                continue;
            }
            std::pair<uint32_t, uint32_t> corners_pair = find_corner(i_com);
            corners[i_com][0] = corners_pair.first;
            corners[i_com][1] = corners_pair.second;
        }
        return;
    }
    //=============================================================================================
    //=============================     find_corner        =======================================
    //=============================================================================================
    std::pair<uint32_t, uint32_t> find_corner(const uint32_t & i_com)
    {
        // given a component will output the pairs of the two most likely labels
        VertexAttributeMap<T> vertex_attribute_map
                                    = boost::get(boost::vertex_bundle, this->main_graph);
        std::vector<T> average_vector(this->dim,0);
        for (uint32_t i_ver = 0;  i_ver < this->components[i_com].size(); i_ver++)
        {
            for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
            {
            average_vector.at(i_dim) += vertex_attribute_map[this->components[i_com][i_ver]].observation[i_dim]
                                *  vertex_attribute_map[this->components[i_com][i_ver]].weight;
            }
        }
        uint32_t indexOfMax = 0;
        for(uint32_t i_dim=1; i_dim < this->dim; i_dim++)
        {
            if(average_vector.at(indexOfMax) < average_vector.at(i_dim))
            {
                indexOfMax = i_dim;
            }
        }
        average_vector[indexOfMax] = -1;
        uint32_t indexOfSndMax = 0;
        for(uint32_t i_dim=1; i_dim < this->dim; i_dim++)
        {
            if(average_vector[indexOfSndMax] < average_vector[i_dim])
            {
                indexOfSndMax = i_dim;
            }
        }
        return std::pair<uint32_t, uint32_t>(indexOfMax, indexOfSndMax);
    }
    //=============================================================================================
    //=============================       SET_CAPACITIES    =======================================
    //=============================================================================================
    inline void set_capacities(const std::vector< std::vector< uint32_t > > & corners)
    {
        VertexDescriptor<T> desc_v;
        EdgeDescriptor   desc_source2v, desc_v2sink, desc_v2source;
        VertexAttributeMap<T> vertex_attribute_map
                = boost::get(boost::vertex_bundle, this->main_graph);
        EdgeAttributeMap<T> edge_attribute_map
                = boost::get(boost::edge_bundle, this->main_graph);
        T cost_B, cost_notB; //the cost of being in B or not B, local for each component
        //----first compute the capacity in sink/node edges------------------------------------
         //#pragma omp parallel for if (this->parameter.parallel) schedule(dynamic)
        for (uint32_t i_com = 0; i_com < this->components.size(); i_com++)
        {
            if (this->saturated_components[i_com])
            {
                continue;
            }
            for (uint32_t i_ver = 0;  i_ver < this->components[i_com].size(); i_ver++)
            {
                desc_v    = this->components[i_com][i_ver];
                // because of the adjacency structure NEVER access edge (source,v) directly!
                desc_v2source = boost::edge(desc_v, this->source,this->main_graph).first;
                desc_source2v = edge_attribute_map(desc_v2source).edge_reverse; //use edge_reverse instead
                desc_v2sink   = boost::edge(desc_v, this->sink,this->main_graph).first;
                cost_B    = 0;
                cost_notB = 0;
                if (vertex_attribute_map(desc_v).weight==0)
                {
                    edge_attribute_map(desc_source2v).capacity = 0;
                    edge_attribute_map(desc_v2sink).capacity   = 0;
                    continue;
                }
                cost_B    += vertex_attribute_map(desc_v).observation[corners[i_com][0]];
                cost_notB += vertex_attribute_map(desc_v).observation[corners[i_com][1]];
                if (cost_B>cost_notB)
                {
                    edge_attribute_map(desc_source2v).capacity = cost_B - cost_notB;
                    edge_attribute_map(desc_v2sink).capacity   = 0.;
                }
                else
                {
                    edge_attribute_map(desc_source2v).capacity = 0.;
                    edge_attribute_map(desc_v2sink).capacity   = cost_notB - cost_B;
                }
            }
        }
        //----then set the vertex to vertex edges ---------------------------------------------
        EdgeIterator<T> i_edg, i_edg_end;
        for (boost::tie(i_edg, i_edg_end) = boost::edges(this->main_graph);
             i_edg != i_edg_end; ++i_edg)
        {
            if (!edge_attribute_map(*i_edg).realEdge)
            {
                continue;
            }
            if (!edge_attribute_map(*i_edg).isActive)
            {
                edge_attribute_map(*i_edg).capacity
                        = edge_attribute_map(*i_edg).weight * this->parameter.reg_strenth;
            }
            else
            {
                edge_attribute_map(*i_edg).capacity = 0;
            }
        }
    }
    //=============================================================================================
    //=================================   COMPUTE_VALUE   =========================================
    //=============================================================================================
    virtual std::pair<std::vector<T>, T> compute_value(const uint32_t & i_com) override
    {
        VertexAttributeMap<T> vertex_attribute_map
                                    = boost::get(boost::vertex_bundle, this->main_graph);
        if (i_com == 0)
        {  // we allocate the space necessary for the component vector at the first read of the component
           this-> componentVector = std::vector<std::vector<T>>(this->components.size());
        }
        std::vector<T> average_vector(this->dim), component_value(this->dim);
        T total_weight = 0;
        for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
        {
            average_vector[i_dim] = 0;
        }
        for (uint32_t ind_ver = 0; ind_ver < this->components[i_com].size(); ++ind_ver)
        {
            for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
            {
            average_vector[i_dim] += vertex_attribute_map[this->components[i_com][ind_ver]].observation[i_dim]
                                *  vertex_attribute_map[this->components[i_com][ind_ver]].weight;
            }
            total_weight += vertex_attribute_map[this->components[i_com][ind_ver]].weight;
            vertex_attribute_map(this->components[i_com][ind_ver]).in_component = i_com;
        }
        this->componentVector[i_com] = average_vector;
        uint32_t indexOfMax = 0;
        for(uint32_t i_dim=1; i_dim < this->dim; i_dim++)
        {
            if(average_vector[indexOfMax] < average_vector[i_dim])
            {
                indexOfMax = i_dim;
            }
        }
        for (uint32_t ind_ver = 0; ind_ver < this->components[i_com].size(); ++ind_ver)
        {
            for(uint32_t i_dim=0; i_dim<this->dim; i_dim++)
            {
               if (i_dim == indexOfMax)
               {
                   component_value[i_dim] = 1;
                   vertex_attribute_map(this->components[i_com][ind_ver]).value[i_dim] = 1;
               }
               else
               {
                   component_value[i_dim] = 0;
                   vertex_attribute_map(this->components[i_com][ind_ver]).value[i_dim] = 0;
               }
            }
        }
        return std::pair<std::vector<T>, T>(component_value, total_weight);
    }
    //=============================================================================================
    //=================================   COMPUTE_MERGE_GAIN   =========================================
    //=============================================================================================
    virtual std::pair<std::vector<T>, T> compute_merge_gain(const VertexDescriptor<T> & comp1
                                         , const VertexDescriptor<T> & comp2) override
    {
        VertexAttributeMap<T> reduced_vertex_attribute_map
                = boost::get(boost::vertex_bundle, this->reduced_graph);
        VertexIndexMap<T> reduced_vertex_vertex_index_map = get(boost::vertex_index, this->reduced_graph);
        std::vector<T> merge_value(this->dim), mergedVector(this->dim);
        T gain = 0;
        // compute the value obtained by mergeing the two connected components
        for(uint32_t i_dim=0; i_dim<this->dim; i_dim++)
        {
            mergedVector[i_dim] = this->componentVector[reduced_vertex_vertex_index_map(comp1)][i_dim]
                                + this->componentVector[reduced_vertex_vertex_index_map(comp2)][i_dim];
        }
        uint32_t indexOfMax = 0;
        for(uint32_t i_dim=1; i_dim < this->dim; i_dim++)
        {
            if(mergedVector[indexOfMax] < mergedVector[i_dim])
            {
                indexOfMax = i_dim;
            }
        }
        for(uint32_t i_dim=0; i_dim<this->dim; i_dim++)
        {
            if (i_dim == indexOfMax)
            {
                merge_value[i_dim] = 1;
            }
            else
            {
                merge_value[i_dim] = 0;
            }
            gain += mergedVector[i_dim] *  merge_value[i_dim]
                  - this->componentVector[reduced_vertex_vertex_index_map(comp1)][i_dim]
                  * reduced_vertex_attribute_map(comp1).value[i_dim]
                  - this->componentVector[reduced_vertex_vertex_index_map(comp2)][i_dim]
                  * reduced_vertex_attribute_map(comp2).value[i_dim];
        }

        return std::pair<std::vector<T>, T>(merge_value, gain);
    }
};
}
