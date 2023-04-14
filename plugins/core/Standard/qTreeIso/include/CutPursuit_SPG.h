#pragma once
#include "CutPursuit.h"
#include "Common.h"

namespace CP {
template <typename T>
class CutPursuit_SPG : public CutPursuit<T>
{
    public:
    ~CutPursuit_SPG(){
            };
    //=============================================================================================
    //=============================     COMPUTE ENERGY  ===========================================
    //=============================================================================================
    virtual std::pair<T,T> compute_energy() override
    {
        VertexAttributeMap<T> vertex_attribute_map
                = boost::get(boost::vertex_bundle, this->main_graph);
        EdgeAttributeMap<T> edge_attribute_map
                = boost::get(boost::edge_bundle, this->main_graph);
        //the first element pair_energy of is the fidelity and the second the penalty
        std::pair<T,T> pair_energy;
        T energy = 0;
        //#pragma omp parallel for private(i_dim) if (this->parameter.parallel) schedule(static) reduction(+:energy,i)
        for (uint32_t ind_ver = 0; ind_ver < this->nVertex; ind_ver++)
        {
            VertexDescriptor<T> i_ver = boost::vertex(ind_ver, this->main_graph);
            for(uint32_t i_dim=0; i_dim<this->dim; i_dim++)
            {
                energy += .5*vertex_attribute_map(i_ver).weight
                        * pow(vertex_attribute_map(i_ver).observation[i_dim]
                            - vertex_attribute_map(i_ver).value[i_dim],2);
            }          
        }
        pair_energy.first = energy;
        energy = 0;
        EdgeIterator<T> i_edg, i_edg_end = boost::edges(this->main_graph).second;
        for (i_edg = boost::edges(this->main_graph).first; i_edg != i_edg_end; ++i_edg)
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
    virtual uint32_t split() override
    { // split the graph by trying to find the best binary partition
      // each components is split into B and notB
      // for each components we associate the value h_1 and h_2 to vertices in B or notB
      // the affectation as well as h_1 and h_2 are computed alternatively
        //tic();
        //--------loading structures---------------------------------------------------------------
        uint32_t nb_comp = this->components.size();
        VertexAttributeMap<T> vertex_attribute_map
                   = boost::get(boost::vertex_bundle, this->main_graph);
        VertexIndexMap<T> vertex_index_map = boost::get(boost::vertex_index, this->main_graph);
        uint32_t saturation;
        //stores wether each vertex is B or not
        std::vector<bool> binary_label(this->nVertex);
        //initialize the binary partition with kmeans
        this->init_labels(binary_label, true);

        //centers is the value of each binary component in the optimal partition
        VectorOfCentroids<T> centers(nb_comp, this->dim);
        //-----main loop----------------------------------------------------------------

        // the optimal flow is iteratively approximated
		T unary_weight = pow(this->parameter.weight_decay, -float(this->parameter.flow_steps));
        for (uint32_t i_step = 0; i_step < this->parameter.flow_steps; i_step++)
        {
			unary_weight = unary_weight * this->parameter.weight_decay;

            //the regularization strength at this step
            //compute h_1 and h_2
            centers = VectorOfCentroids<T>(nb_comp, this->dim);
            this->compute_centers(centers, nb_comp,binary_label);
            this->set_capacities(centers, unary_weight);

            // update the capacities of the flow graph
            boost::boykov_kolmogorov_max_flow(
                       this->main_graph,
                       get(&EdgeAttribute<T>::capacity        , this->main_graph),
                       get(&EdgeAttribute<T>::residualCapacity, this->main_graph),
                       get(&EdgeAttribute<T>::edge_reverse     , this->main_graph),
                       get(&VertexAttribute<T>::color         , this->main_graph),
                       get(boost::vertex_index                , this->main_graph),
                       this->source,
                       this->sink);

            for (uint32_t ind_com = 0; ind_com < nb_comp; ind_com++)
            {
                if (this->saturated_components[ind_com])
                {
                    continue;
                }
	
                for (uint32_t i_ver = 0;  i_ver < this->components[ind_com].size(); i_ver++)
                {	
                    binary_label[vertex_index_map(this->components[ind_com][i_ver])]
                          = (vertex_attribute_map(this->components[ind_com][i_ver]).color
                          == vertex_attribute_map(this->sink).color);

                 }
             }
        }
        saturation = this->activate_edges(false);
        return saturation;
    }
    //=============================================================================================
    //=============================      INIT_L2 ====== ===========================================
    //=============================================================================================
    inline void init_labels(std::vector<bool> & binary_label, bool spatial_part)
    { //-----initialize the labelling for each components with kmeans------------------------------
        VertexAttributeMap<T> vertex_attribute_map 
                = boost::get(boost::vertex_bundle, this->main_graph);
        VertexIndexMap<T> vertex_index_map = boost::get(boost::vertex_index, this->main_graph);
        uint32_t nb_comp = this->components.size();
        // ind_com;
        //#pragma omp parallel for private(ind_com) //if (nb_comp>=8) schedule(dynamic)
		int dim_spat;
		if (spatial_part)
		{
			dim_spat = this->dim-0;
		}
		else
		{
			dim_spat = this->dim;
		}

        #ifdef OPENMP
		#pragma omp parallel for if (nb_comp >= omp_get_num_threads()) schedule(dynamic) 
        #endif
        for (uint32_t ind_com = 0; ind_com < nb_comp; ind_com++)
        {

			std::vector< std::vector<T> > kernels(2, std::vector<T>(this->dim));
            T total_weight[2];
            T best_energy;
            T current_energy;
            uint32_t comp_size = this->components[ind_com].size();
            std::vector<bool> potential_label(comp_size);    
            std::vector<T> energy_array(comp_size);            
            if (this->saturated_components[ind_com] || comp_size <= 1)
            {
                continue;
            }
            for (uint32_t init_kmeans = 0; init_kmeans < this->parameter.kmeans_resampling; init_kmeans++)
            {//proceed to several initilialisation of kmeans and pick up the best one
                //----- initialization with KM++ ------------------
            uint32_t first_kernel  = std::rand() % comp_size, second_kernel = 0; // first kernel attributed
			for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
            {
				kernels[0][i_dim] = vertex_attribute_map(this->components[ind_com][first_kernel ]).observation[i_dim];
			}
            best_energy = 0; //now compute the square distance of each vertex to this kernel
            #ifdef OPENMP
            #pragma omp parallel for if (nb_comp < omp_get_num_threads()) shared(best_energy) schedule(static) 
            #endif
			for (uint32_t i_ver = 0;  i_ver < comp_size; i_ver++)
            {
            	energy_array[i_ver] = 0;
                for(uint32_t i_dim=0; i_dim < dim_spat; i_dim++)
                {
                	energy_array[i_ver] += pow(vertex_attribute_map(this->components[ind_com][i_ver]).observation[i_dim]- kernels[0][i_dim],2) * vertex_attribute_map(this->components[ind_com][i_ver]).weight;				
                }
				best_energy += energy_array[i_ver];
             } // we now generate a random number to determinate which node will be the second kernel             
			 T random_sample = ((T)(rand())) / ((T)(RAND_MAX));
             current_energy = best_energy * random_sample;
             for (uint32_t i_ver = 0;  i_ver < comp_size; i_ver++)
             {
             	current_energy -= energy_array[i_ver];
                if (current_energy < 0)
                { //we have selected the second kernel
                	second_kernel = i_ver;        						
                    break;
                }
            }
            for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
            { // now fill the second kernel
            	kernels[1][i_dim] = vertex_attribute_map(this->components[ind_com][second_kernel]).observation[i_dim];
			}
            //----main kmeans loop-----
            for (uint32_t ite_kmeans = 0; ite_kmeans < this->parameter.kmeans_ite; ite_kmeans++)
            {
                //--affectation step: associate each node with its closest kernel-------------------
                #ifdef OPENMP
                #pragma omp parallel for if (nb_comp < omp_get_num_threads()) shared(potential_label) schedule(static) 
                #endif
				for (uint32_t i_ver = 0;  i_ver < comp_size; i_ver++)
                {
                    std::vector<T> distance_kernels(2);
                    for(uint32_t i_dim=0; i_dim < dim_spat; i_dim++)
                    {
                    	distance_kernels[0] += pow(vertex_attribute_map(this->components[ind_com][i_ver]).observation[i_dim]- kernels[0][i_dim],2);
                        distance_kernels[1] += pow(vertex_attribute_map(this->components[ind_com][i_ver]).observation[i_dim] - kernels[1][i_dim],2);
                     }
                    potential_label[i_ver] = distance_kernels[0] > distance_kernels[1];
                }
                //-----computation of the new kernels----------------------------
                 total_weight[0] = 0.;
                 total_weight[1] = 0.;
                    for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
                    {
                       kernels[0][i_dim] = 0;
                       kernels[1][i_dim] = 0;
                    }
                    #ifdef OPENMP
					#pragma omp parallel for if (nb_comp < omp_get_num_threads()) shared(potential_label) schedule(static) 
                    #endif
                    for (uint32_t i_ver = 0;  i_ver < comp_size; i_ver++)
                    {
                        if (vertex_attribute_map(this->components[ind_com][i_ver]).weight==0)
                        {
                            continue;
                        }
                        if (potential_label[i_ver])
                        {
                            total_weight[0] += vertex_attribute_map(this->components[ind_com][i_ver]).weight;
                            for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
                            {
                                kernels[0][i_dim] += vertex_attribute_map(this->components[ind_com][i_ver]).observation[i_dim]
                                                  * vertex_attribute_map(this->components[ind_com][i_ver]).weight ;
                             }
                         }
                         else
                         {
                            total_weight[1] += vertex_attribute_map(this->components[ind_com][i_ver]).weight;
                            for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
                            {
                                kernels[1][i_dim] += vertex_attribute_map(this->components[ind_com][i_ver]).observation[i_dim]
                                                  * vertex_attribute_map(this->components[ind_com][i_ver]).weight;
                            }
                         }
                    }  

                    if ((total_weight[0] == 0)||(total_weight[1] == 0))
                    {
                        break;	
                    }
                    for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
                    {
                        kernels[0][i_dim] = kernels[0][i_dim] / total_weight[0];
                        kernels[1][i_dim] = kernels[1][i_dim] / total_weight[1];
                    }
                }
                //----compute the associated energy ------
                current_energy = 0;
                #ifdef OPENMP
				#pragma omp parallel for if (nb_comp < omp_get_num_threads()) shared(potential_label) schedule(static) 
                #endif
                for (uint32_t i_ver = 0;  i_ver < comp_size; i_ver++)
                {
                    for(uint32_t i_dim=0; i_dim < dim_spat; i_dim++)
                    {
                       if (potential_label[i_ver])
                       {
                       current_energy += pow(vertex_attribute_map(this->components[ind_com][i_ver]).observation[i_dim]
                                        - kernels[0][i_dim],2) * vertex_attribute_map(this->components[ind_com][i_ver]).weight;
                       }
                       else
                       {
                        current_energy += pow(vertex_attribute_map(this->components[ind_com][i_ver]).observation[i_dim]
                                        - kernels[1][i_dim],2) * vertex_attribute_map(this->components[ind_com][i_ver]).weight;
                        }
                   }
                }                  
                if (current_energy < best_energy)
                {
                    best_energy = current_energy;
                    for (uint32_t i_ver = 0;  i_ver < comp_size; i_ver++)
                    {
                        binary_label[vertex_index_map(this->components[ind_com][i_ver])] = potential_label[i_ver];
                    }
                }
            }
        }
    }
    //=============================================================================================
    //=============================  COMPUTE_CENTERS_L2  ==========================================
    //=============================================================================================
    inline void compute_centers(VectorOfCentroids<T> & centers, const uint32_t & nb_comp
                               , const std::vector<bool> & binary_label)
    {
        //compute for each component the values of h_1 and h_2
        #ifdef OPENMP
        #pragma omp parallel for if (nb_comp >= omp_get_num_threads()) schedule(dynamic)
        #endif
        for (uint32_t ind_com = 0; ind_com < nb_comp; ind_com++)
        {
            if (this->saturated_components[ind_com])
            {
                continue;
            }
            compute_center(centers.centroids[ind_com], ind_com, binary_label);
        }
        return;
    }

    //=============================================================================================
    //=============================  COMPUTE_CENTER_L2  ==========================================
    //=============================================================================================
    inline void compute_center( std::vector< std::vector<T> > & center, const uint32_t & ind_com
                             , const std::vector<bool> & binary_label)
    {
        //compute for each component the values of the centroids corresponding to the optimal binary partition
        VertexAttributeMap<T> vertex_attribute_map
                = boost::get(boost::vertex_bundle, this->main_graph);
        VertexIndexMap<T> vertex_index_map = boost::get(boost::vertex_index, this->main_graph);
        T total_weight[2];
        total_weight[0] = 0.;
        total_weight[1] = 0.;

        //#pragma omp parallel for if (this->parameter.parallel)
        for (uint32_t i_ver = 0;  i_ver < this->components[ind_com].size(); i_ver++)
        {
            if (vertex_attribute_map(this->components[ind_com][i_ver]).weight==0)
            {
                continue;
            }
            if (binary_label[vertex_index_map(this->components[ind_com][i_ver])])
            {
                total_weight[0] += vertex_attribute_map(this->components[ind_com][i_ver]).weight;
                for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
                {
                    center[0][i_dim] += vertex_attribute_map(this->components[ind_com][i_ver]).observation[i_dim]
                                      * vertex_attribute_map(this->components[ind_com][i_ver]).weight ;
                 }
             }
             else
             {
                total_weight[1] += vertex_attribute_map(this->components[ind_com][i_ver]).weight;
                for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
                {
                    center[1][i_dim] += vertex_attribute_map(this->components[ind_com][i_ver]).observation[i_dim]
                                      * vertex_attribute_map(this->components[ind_com][i_ver]).weight;
                }
             }
        }
        if ((total_weight[0] == 0)||(total_weight[1] == 0))
        {
            //the component is saturated
            //this->saturateComponent(ind_com, false);
            for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
            {
                center[0][i_dim] = vertex_attribute_map(this->components[ind_com][0]).value[i_dim];
                center[1][i_dim] = vertex_attribute_map(this->components[ind_com][0]).value[i_dim];
            }
        }
        else
        {
            for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
            {
                center[0][i_dim] = center[0][i_dim] / total_weight[0];
                center[1][i_dim] = center[1][i_dim] / total_weight[1];
             }
        }


    return;
    }
    //=============================================================================================
    //=============================   SET_CAPACITIES     ==========================================
    //=============================================================================================
    inline void set_capacities(const VectorOfCentroids<T> & centers, T unary_weight)
    {
        VertexAttributeMap<T> vertex_attribute_map
                = boost::get(boost::vertex_bundle, this->main_graph);
        EdgeAttributeMap<T> edge_attribute_map
                = boost::get(boost::edge_bundle, this->main_graph);
        //----first compute the capacity in sink/node edges------------------------------------
        //#pragma omp parallel for if (this->parameter.parallel) schedule(dynamic)
	    uint32_t nb_comp = this->components.size();
        #ifdef OPENMP
		#pragma omp parallel for if (nb_comp >= omp_get_num_threads()) schedule(dynamic)
        #endif
        for (uint32_t ind_com = 0; ind_com < nb_comp; ind_com++)
        {	
	VertexDescriptor<T> desc_v;
		    EdgeDescriptor   desc_source2v, desc_v2sink, desc_v2source;
		    T cost_B, cost_notB; //the cost of being in B or not B, local for each component
            if (this->saturated_components[ind_com])
            {
                continue;
            }
            for (uint32_t i_ver = 0;  i_ver < this->components[ind_com].size(); i_ver++)
            {
                desc_v    = this->components[ind_com][i_ver];
                // because of the adjacency structure NEVER access edge (source,v) directly!
                desc_v2source = boost::edge(desc_v, this->source,this->main_graph).first;
                desc_source2v = edge_attribute_map(desc_v2source).edge_reverse; //use edge_reverse instead
                desc_v2sink   = boost::edge(desc_v, this->sink,this->main_graph).first;
                cost_B    = 0;
                cost_notB = 0;
                if (vertex_attribute_map(desc_v).weight==0)
                {   //no observation - no cut
                    edge_attribute_map(desc_source2v).capacity = 0;
                    edge_attribute_map(desc_v2sink).capacity   = 0;
                    continue;
                }
                for(uint32_t i_dim=0; i_dim < this->dim; i_dim++)
                {
                   cost_B    += 0.5*vertex_attribute_map(desc_v).weight
                              * (pow(centers.centroids[ind_com][0][i_dim],2) - 2 * (centers.centroids[ind_com][0][i_dim]
                              * vertex_attribute_map(desc_v).observation[i_dim]));
                   cost_notB += 0.5*vertex_attribute_map(desc_v).weight
                              * (pow(centers.centroids[ind_com][1][i_dim],2) - 2 * (centers.centroids[ind_com][1][i_dim]
                              * vertex_attribute_map(desc_v).observation[i_dim]));
                
				}
                if (cost_B>cost_notB)
                {
                    edge_attribute_map(desc_source2v).capacity = (cost_B - cost_notB) ;
                    edge_attribute_map(desc_v2sink).capacity   = 0.;
                }
                else
                {
                    edge_attribute_map(desc_source2v).capacity = 0.;
                    edge_attribute_map(desc_v2sink).capacity   = (cost_notB - cost_B) ;
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
                        = edge_attribute_map(*i_edg).weight * this->parameter.reg_strenth / unary_weight;
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
    virtual std::pair<std::vector<T>, T> compute_value(const uint32_t & ind_com) override
    {
        VertexAttributeMap<T> vertex_attribute_map
                                    = boost::get(boost::vertex_bundle, this->main_graph);
        T total_weight = 0;
        std::vector<T> compValue(this->dim);
        std::fill((compValue.begin()),(compValue.end()),0);
        #ifdef OPENMP
        #pragma omp parallel for if (this->parameter.parallel) schedule(static)
        #endif
        for (uint32_t ind_ver = 0; ind_ver < this->components[ind_com].size(); ++ind_ver)
        {
            total_weight += vertex_attribute_map(this->components[ind_com][ind_ver]).weight;
            for(uint32_t i_dim=0; i_dim<this->dim; i_dim++)
            {
                compValue[i_dim] += vertex_attribute_map(this->components[ind_com][ind_ver]).observation[i_dim]
                                       * vertex_attribute_map(this->components[ind_com][ind_ver]).weight;
            }
            vertex_attribute_map(this->components[ind_com][ind_ver]).in_component = ind_com;
         }
         for(uint32_t i_dim=0; i_dim<this->dim; i_dim++)
         {
            compValue[i_dim] = compValue[i_dim] / total_weight;
         }
         for (uint32_t ind_ver = 0; ind_ver < this->components[ind_com].size(); ++ind_ver)
         {
            for(uint32_t i_dim=0; i_dim<this->dim; i_dim++)
            {
                vertex_attribute_map(this->components[ind_com][ind_ver]).value[i_dim] = compValue[i_dim];
            }
         }
        return std::pair<std::vector<T>, T>(compValue, total_weight);
    }
    //=============================================================================================
    //=================================   COMPUTE_MERGE_GAIN   =========================================
    //=============================================================================================
    virtual std::pair<std::vector<T>, T> compute_merge_gain(const VertexDescriptor<T> & comp1
                                             , const VertexDescriptor<T> & comp2) override
    {
        VertexAttributeMap<T> reduced_vertex_attribute_map
                = boost::get(boost::vertex_bundle, this->reduced_graph);
        std::vector<T> merge_value(this->dim);
        T gain = 0;
        // compute the value obtained by mergeing the two connected components
        for(uint32_t i_dim=0; i_dim<this->dim; i_dim++)
        {
            merge_value[i_dim] =
                    (reduced_vertex_attribute_map(comp1).weight *
                     reduced_vertex_attribute_map(comp1).value[i_dim]
                    +reduced_vertex_attribute_map(comp2).weight *
                     reduced_vertex_attribute_map(comp2).value[i_dim])
                   /(reduced_vertex_attribute_map(comp1).weight
                    +reduced_vertex_attribute_map(comp2).weight);
            gain += 0.5 * (pow(merge_value[i_dim],2)
                  * (reduced_vertex_attribute_map(comp1).weight
                    +reduced_vertex_attribute_map(comp2).weight)
                  - pow(reduced_vertex_attribute_map(comp1).value[i_dim],2)
                  * reduced_vertex_attribute_map(comp1).weight
                  - pow(reduced_vertex_attribute_map(comp2).value[i_dim],2)
                  * reduced_vertex_attribute_map(comp2).weight);
        }
        return std::pair<std::vector<T>, T>(merge_value, gain);
    }
};
}
