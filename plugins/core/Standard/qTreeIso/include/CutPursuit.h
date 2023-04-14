#pragma once
#include "Graph.h"
#include <math.h>
#include <queue>
#include <iostream>
#include <fstream>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
namespace CP {

template <typename T>
struct CPparameter
{
    T   reg_strenth;  //regularization strength, multiply the edge weight
    uint32_t cutoff;  //minimal component size
    uint32_t flow_steps; //number of steps in the optimal binary cut computation
    uint32_t kmeans_ite; //number of iteration in the kmeans sampling
    uint32_t kmeans_resampling; //number of kmeans re-intilialization
    uint32_t verbose; //verbosity
    uint32_t max_ite_main; //max number of iterations in the main loop
    bool backward_step; //indicates if a backward step should be performed
    double stopping_ratio; //when (E(t-1) - E(t) / (E(0) - E(t)) is too small, the algorithm stops
    fidelityType fidelity; //the fidelity function
    double smoothing; //smoothing term (for Kl divergence only)
    bool parallel; //enable/disable parrallelism
	T weight_decay; //for continued optimization of the flow steps
};

template <typename T>
class CutPursuit
{
    public:
    Graph<T> main_graph; //the Graph structure containing the main structure
    Graph<T> reduced_graph; //the reduced graph whose vertices are the connected component
    std::vector<std::vector<VertexDescriptor<T>>> components; //contains the list of the vertices in each component
    std::vector<VertexDescriptor<T>> root_vertex; //the root vertex for each connected components
    std::vector<bool> saturated_components; //is the component saturated (uncuttable)
    std::vector<std::vector<EdgeDescriptor>> borders; //the list of edges forming the borders between the connected components
    VertexDescriptor<T> source; //source vertex for graph cut
    VertexDescriptor<T> sink; //sink vertex
    uint32_t dim;     // dimension of the data
    uint32_t nVertex; // number of data point
    uint32_t nEdge;   // number of edges between vertices (not counting the edge to source/sink)
    CP::VertexIterator<T> lastIterator; //iterator pointing to the last vertex which is neither sink nor source
    CPparameter<T> parameter;
    public:      
    CutPursuit(uint32_t nbVertex = 1)
    {
        this->main_graph     = Graph<T>(nbVertex);
        this->reduced_graph  = Graph<T>(1);
        this->components     = std::vector<std::vector<VertexDescriptor<T>>>(1);
        this->root_vertex    = std::vector<VertexDescriptor<T>>(1,0);
        this->saturated_components = std::vector<bool>(1,false);
        this->source         = VertexDescriptor<T>();
        this->sink           = VertexDescriptor<T>();
        this->dim            = 1;
        this->nVertex        = 1;
        this->nEdge          = 0;
        this->parameter.flow_steps  = 3;
        this->parameter.kmeans_ite  = 5;
        this->parameter.kmeans_resampling = 3;
        this->parameter.verbose = 2;
        this->parameter.max_ite_main = 6;
        this->parameter.backward_step = true;
        this->parameter.stopping_ratio = 0.0001;
        this->parameter.fidelity = L2;
        this->parameter.smoothing = 0.1;
        this->parameter.parallel = true;
		this->parameter.weight_decay = 0.7;
    }
    virtual ~CutPursuit(){
    };  
    //=============================================================================================
    std::pair<std::vector<T>, std::vector<T>> run()
    {
        //first initilialize the structure
        this->initialize();
        if (this->parameter.verbose > 0)
        {
            std::cout << "Graph "  << boost::num_vertices(this->main_graph) << " vertices and "
             <<   boost::num_edges(this->main_graph)  << " edges and observation of dimension "
             << this->dim << '\n';
        }
        T energy_zero = this->compute_energy().first; //energy with 1 component
        T old_energy = energy_zero; //energy at the previous iteration
        //vector with time and energy, useful for benchmarking
        std::vector<T> energy_out(this->parameter.max_ite_main ),time_out(this->parameter.max_ite_main);
        TimeStack ts; ts.tic();
        //the main loop
        for (uint32_t ite_main = 1; ite_main <= this->parameter.max_ite_main; ite_main++)
        {
            //--------those two lines are the whole iteration-------------------------
            uint32_t saturation = this->split(); //compute optimal binary partition
            this->reduce(); //compute the new reduced graph
            //-------end of the iteration - rest is stopping check and display------
            std::pair<T,T> energy = this->compute_energy();
            energy_out.push_back((energy.first + energy.second));
            time_out.push_back(ts.tocDouble());
            if (this->parameter.verbose > 1)
            {
                printf("Iteration %3i - %4i components - ", ite_main, (int)this->components.size());
                printf("Saturation %5.1f %% - ",100*saturation / (double) this->nVertex);
                switch (this->parameter.fidelity)
                {
                    case L2:
                    {
                        printf("Quadratic Energy %4.3f %% - ", 100 * (energy.first + energy.second) / energy_zero);
                        break;
                    }
                    case linear:
                    {
                        printf("Linear Energy %10.1f - ", energy.first + energy.second);
                        break;
                    }
                    case KL:
                    {
                        printf("KL Energy %4.3f %% - ", 100 * (energy.first + energy.second) / energy_zero);
                        break;
                    }
       case SPG:
                    {
                        printf("Quadratic Energy %4.3f %% - ", 100 * (energy.first + energy.second) / energy_zero);
                        break;
                    }
                }
                std::cout << "Timer  " << ts.toc() << std::endl;
            }
            //----stopping checks-----
            if (saturation == (double) this->nVertex)
            {   //all components are saturated
                if (this->parameter.verbose > 1)
                {
                    std::cout << "All components are saturated" << std::endl;
                }
                break;
            }
            if ((old_energy - energy.first - energy.second) / (old_energy)
               < this->parameter.stopping_ratio)
            {   //relative energy progress stopping criterion
                if (this->parameter.verbose > 1)
                {
                    std::cout << "Stopping criterion reached" << std::endl;
                }
                break;
            }
            if (ite_main>=this->parameter.max_ite_main)
            {   //max number of iteration
                if (this->parameter.verbose > 1)
                {
                    std::cout << "Max number of iteration reached" << std::endl;
                }
                break;
            }
            old_energy = energy.first + energy.second;
        }
        if (this->parameter.cutoff > 0)
        {
            this->cutoff();
        }
        return std::pair<std::vector<T>, std::vector<T>>(energy_out, time_out);
    }
    //=============================================================================================
    //=========== VIRTUAL METHODS DEPENDING ON THE CHOICE OF FIDELITY FUNCTION =====================
    //=============================================================================================
    //
    //=============================================================================================
    //=============================        SPLIT        ===========================================
    //=============================================================================================
    virtual uint32_t split()
    {
        //compute the optimal binary partition
        return 0;
    }
    //=============================================================================================
    //================================     compute_energy_L2      ====================================
    //=============================================================================================
    virtual std::pair<T,T> compute_energy()
    {
        //compute the current energy
        return std::pair<T,T>(0,0);
    }
    //=============================================================================================
    //=================================   COMPUTE_VALUE   =========================================
    //=============================================================================================
    virtual std::pair<std::vector<T>, T> compute_value(const uint32_t & ind_com)
    {
        //compute the optimal the values associated with the current partition
        return std::pair<std::vector<T>, T>(std::vector<T>(0),0);
    }
//=============================================================================================
//=================================   COMPUTE_MERGE_GAIN   =========================================
//=============================================================================================
    virtual std::pair<std::vector<T>, T> compute_merge_gain(const VertexDescriptor<T> & comp1
                                                          , const VertexDescriptor<T> & comp2)
    {
        //compute the gain of mergeing two connected components
        return std::pair<std::vector<T>, T>(std::vector<T>(0),0);
    }
    //=============================================================================================
    //========================== END OF VIRTUAL METHODS ===========================================
    //=============================================================================================
    //
    //=============================================================================================
    //=============================     INITIALIZE      ===========================================
    //=============================================================================================
    void initialize()
    {
        //build the reduced graph with one component, fill the first vector of components
        //and add the sink and source nodes
        VertexIterator<T> ite_ver, ite_ver_end;
        EdgeAttributeMap<T> edge_attribute_map
            = boost::get(boost::edge_bundle, this->main_graph);
        this->components[0]  = std::vector<VertexDescriptor<T>> (0);//(this->nVertex);
        this->root_vertex[0] = *boost::vertices(this->main_graph).first;
        this->nVertex = boost::num_vertices(this->main_graph);
        this->nEdge   = boost::num_edges(this->main_graph);
        //--------compute the first reduced graph----------------------------------------------------------
        for (boost::tie(ite_ver, ite_ver_end) = boost::vertices(this->main_graph);
             ite_ver != ite_ver_end; ++ite_ver)
        {
            this->components[0].push_back(*ite_ver);
        }
        this->lastIterator = ite_ver;
        this->compute_value(0);
        //--------build the link to source and sink--------------------------------------------------------
        this->source = boost::add_vertex(this->main_graph);
        this->sink   = boost::add_vertex(this->main_graph);
        uint32_t eIndex = boost::num_edges(this->main_graph);
        ite_ver = boost::vertices(this->main_graph).first;
        for (uint32_t ind_ver = 0;  ind_ver < this->nVertex ; ind_ver++)
        {
            // note that source and edge will have many nieghbors, and hence boost::edge should never be called to get
            // the in_edge. use the out_edge and then reverse_Edge
            addDoubledge<T>(this->main_graph, this->source, boost::vertex(ind_ver, this->main_graph), 0.,
                         eIndex, edge_attribute_map , false);
            eIndex +=2;
            addDoubledge<T>(this->main_graph, boost::vertex(ind_ver, this->main_graph), this->sink, 0.,
                         eIndex, edge_attribute_map, false);
            eIndex +=2;
            ++ite_ver;
        }

    }
    //=============================================================================================
    //================================  COMPUTE_REDUCE_VALUE  ====================================
    //=============================================================================================
    void compute_reduced_value()
    {
        for (uint32_t ind_com = 0;  ind_com < this->components.size(); ++ind_com)
        {   //compute the reduced value of each component
            compute_value(ind_com);
        }
    }
    //=============================================================================================
    //=============================   ACTIVATE_EDGES     ==========================================
    //=============================================================================================
    uint32_t activate_edges(bool allows_saturation = true)
    {   //this function analyzes the optimal binary partition to detect:
        //- saturated components (i.e. uncuttable)
        //- new activated edges
        VertexAttributeMap<T> vertex_attribute_map
            = boost::get(boost::vertex_bundle, this->main_graph);
        EdgeAttributeMap<T> edge_attribute_map
            = boost::get(boost::edge_bundle, this->main_graph);
        //saturation is the proportion of nodes in saturated components
        uint32_t saturation = 0;
        uint32_t nb_comp = this->components.size();
        //---- first check if the component are saturated-------------------------
        //#pragma omp parallel for if (this->parameter.parallel) schedule(dynamic)
        for (uint32_t ind_com = 0; ind_com < nb_comp; ind_com++)
        {
            if (this->saturated_components[ind_com])
            {   //ind_com is saturated, we increement saturation by ind_com size
                saturation += this->components[ind_com].size();
                continue;
            }
            std::vector<T> totalWeight(2,0);
            for (uint32_t ind_ver = 0;  ind_ver < this->components[ind_com].size(); ind_ver++)
            {
                bool isSink
                        = (vertex_attribute_map(this->components[ind_com][ind_ver]).color
                        ==  vertex_attribute_map(this->sink).color);
                if (isSink)
                {
                    totalWeight[0] += vertex_attribute_map(this->components[ind_com][ind_ver]).weight;
                }
                else
                {
                   totalWeight[1] += vertex_attribute_map(this->components[ind_com][ind_ver]).weight;
                }
            }
            if (allows_saturation && ((totalWeight[0] == 0)||(totalWeight[1] == 0)))
            {
                //the component is saturated
                this->saturateComponent(ind_com);
                saturation += this->components[ind_com].size();
            }
        }
        //----check which edges have been activated----
        EdgeIterator<T> ite_edg, ite_edg_end;
        uint32_t color_v1, color_v2, color_combination;
        for (boost::tie(ite_edg, ite_edg_end) = boost::edges(this->main_graph);
             ite_edg != ite_edg_end; ++ite_edg)
        {
            if (!edge_attribute_map(*ite_edg).realEdge )
            {
                continue;
            }
            color_v1 = vertex_attribute_map(boost::source(*ite_edg, this->main_graph)).color;
            color_v2 = vertex_attribute_map(boost::target(*ite_edg, this->main_graph)).color;
            //color_source = 0, color_sink = 4, uncolored = 1
            //we want an edge when a an interface source/sink
            //this corresponds to a sum of 4
            //for the case of uncolored nodes we arbitrarily chose source-uncolored
            color_combination = color_v1 + color_v2;
            if ((color_combination == 0)||(color_combination == 2)||(color_combination == 2)
              ||(color_combination == 8))
            {   //edge between two vertices of the same color
                continue;
            }
            //the edge is active!
            edge_attribute_map(*ite_edg).isActive = true;
            edge_attribute_map(*ite_edg).capacity = 0;
            vertex_attribute_map(boost::source(*ite_edg, this->main_graph)).isBorder = true;
            vertex_attribute_map(boost::target(*ite_edg, this->main_graph)).isBorder = true;
        }
        return saturation;
    }

    //=============================================================================================
    //=============================        REDUCE       ===========================================
    //=============================================================================================
    void reduce()
    {   //compute the reduced graph, and if need be performed a backward check
        this->compute_connected_components();
        if (this->parameter.backward_step)
        {   //compute the structure of the reduced graph        
            this->compute_reduced_graph();
            //check for beneficial merges
            this->merge(false);
        }
        else
        {   //compute only the value associated to each connected components
            this->compute_reduced_value();
        }
    }
    //=============================================================================================
    //==============================  compute_connected_components=========================================
    //=============================================================================================
    void compute_connected_components()
    {  //this function compute the connected components of the graph with active edges removed
        //the boolean vector indicating wether or not the edges and vertices have been seen already
        //the root is the first vertex of a component
        //this function is written such that the new components are appended at the end of components
        //this allows not to recompute saturated component
        VertexAttributeMap<T> vertex_attribute_map
            = boost::get(boost::vertex_bundle, this->main_graph);
        VertexIndexMap<T> vertex_index_map =get(boost::vertex_index, this->main_graph);
        //indicate which edges and nodes have been seen already by the dpsearch
        std::vector<bool> edges_seen (this->nEdge, false);
        std::vector<bool> vertices_seen (this->nVertex+2, false);
        vertices_seen[vertex_index_map(this->source)] = true;
        vertices_seen[vertex_index_map(this->sink)]   = true;
        //-------- start with the known roots------------------------------------------------------
        //#pragma omp parallel for if (this->parameter.parallel) schedule(dynamic)
        for (uint32_t ind_com = 0; ind_com < this->root_vertex.size(); ind_com++)
        {
            VertexDescriptor<T> root = this->root_vertex[ind_com]; //the first vertex of the component
            if (this->saturated_components[ind_com])
            {   //this component is saturated, we don't need to recompute it
                for (uint32_t ind_ver = 0; ind_ver < this->components[ind_com].size(); ++ind_ver)
                {
                    vertices_seen[vertex_index_map(this->components[ind_com][ind_ver])] = true;
                }
            }
            else
            {   //compute the new content of this component
                this->components.at(ind_com) = connected_comp_from_root(root, this->components.at(ind_com).size()
                                          , vertices_seen , edges_seen);
             }
        }
        //----now look for components that did not already exists----
        VertexIterator<T> ite_ver;
        for (ite_ver = boost::vertices(this->main_graph).first;
             ite_ver != this->lastIterator; ++ite_ver)
        {
            if (vertices_seen[vertex_index_map(*ite_ver)])
            {
                 continue;
            } //this vertex is not currently in a connected component
            VertexDescriptor<T> root = *ite_ver; //we define it as the root of a new component
            uint32_t current_component_size =
                    this->components[vertex_attribute_map(root).in_component].size();
            this->components.push_back(
                    connected_comp_from_root(root, current_component_size
                  , vertices_seen, edges_seen));
            this->root_vertex.push_back(root);
            this->saturated_components.push_back(false);
        }
        this->components.shrink_to_fit();
    }
    //=============================================================================================
    //==============================  CONNECTED_COMP_FROM_ROOT=========================================
    //=============================================================================================
    inline std::vector<VertexDescriptor<T>> connected_comp_from_root(const VertexDescriptor<T> & root
                , const uint32_t & size_comp, std::vector<bool> & vertices_seen , std::vector<bool> & edges_seen)
    {
        //this function compute the connected component of the graph with active edges removed
        // associated with the root ROOT by performing a depth search first
        EdgeAttributeMap<T> edge_attribute_map
             = boost::get(boost::edge_bundle, this->main_graph);
        VertexIndexMap<T> vertex_index_map = get(boost::vertex_index, this->main_graph);
        EdgeIndexMap<T>   edge_index_map = get(&EdgeAttribute<T>::index, this->main_graph);
        std::vector<VertexDescriptor<T>> vertices_added; //the vertices in the current connected component
        // vertices_added contains the vertices that have been added to the current coomponent
        vertices_added.reserve(size_comp);
        //heap_explore contains the vertices to be added to the current component
        std::vector<VertexDescriptor<T>> vertices_to_add;
        vertices_to_add.reserve(size_comp);
        VertexDescriptor<T> vertex_current; //the node being consideed
        EdgeDescriptor      edge_current, edge_reverse; //the edge being considered
        //fill the heap with the root node
        vertices_to_add.push_back(root);
        while (vertices_to_add.size()>0)
        {   //as long as there are vertices left to add
            vertex_current = vertices_to_add.back(); //the current node is the last node to add
            vertices_to_add.pop_back(); //remove the current node from the vertices to add
            if (vertices_seen[vertex_index_map(vertex_current)])
            {   //this vertex has already been treated
                continue;
            }
            vertices_added.push_back(vertex_current); //we add the current node
            vertices_seen[vertex_index_map(vertex_current)] = true ; //and flag it as seen
            //----we now explore the neighbors of current_node
            typename boost::graph_traits<Graph<T>>::out_edge_iterator ite_edg, ite_edg_end;
            for (boost::tie(ite_edg,ite_edg_end) = boost::out_edges(vertex_current, this->main_graph);
                ite_edg !=  ite_edg_end; ++ite_edg)
                {   //explore edges leaving current_node
                    edge_current = *ite_edg;
                    if (edge_attribute_map(*ite_edg).isActive || (edges_seen[edge_index_map(edge_current)]))
                    {   //edge is either active or treated, we skip it
                        continue;
                    }
                    //the target of this edge is a node to add
                    edge_reverse = edge_attribute_map(edge_current).edge_reverse;
                    edges_seen[edge_index_map(edge_current)] = true;
                    edges_seen[edge_index_map(edge_reverse)] = true;
                    vertices_to_add.push_back(boost::target(edge_current, this->main_graph));
               }
            }
            vertices_added.shrink_to_fit();
            return vertices_added;
    }
    //=============================================================================================
    //================================  COMPUTE_REDUCE_GRAPH   ====================================
    //=============================================================================================
    void compute_reduced_graph()
    {   //compute the adjacency structure between components as well as weight and value of each component
        //this is stored in the reduced graph structure
        EdgeAttributeMap<T> edge_attribute_map
            = boost::get(boost::edge_bundle, this->main_graph);
        VertexAttributeMap<T> vertex_attribute_map
            = boost::get(boost::vertex_bundle, this->main_graph);
        this->reduced_graph = Graph<T>(this->components.size());
        VertexAttributeMap<T> component_attribute_map = boost::get(boost::vertex_bundle, this->reduced_graph);
        //----fill the value sof the reduced graph----
        #ifdef OPENMP
        #pragma omp parallel for schedule(dynamic) 
        #endif
        for (uint32_t ind_com = 0;  ind_com < this->components.size(); ind_com++)
        {
            std::pair<std::vector<T>, T> component_values_and_weight = this->compute_value(ind_com);
            //----fill the value and weight field of the reduced graph-----------------------------
            VertexDescriptor<T> reduced_vertex = boost::vertex(ind_com, this->reduced_graph);
            component_attribute_map[reduced_vertex] = VertexAttribute<T>(this->dim);
            component_attribute_map(reduced_vertex).weight
                    = component_values_and_weight.second;
            for(uint32_t i_dim=0; i_dim<this->dim; i_dim++)
            {
                component_attribute_map(reduced_vertex).value[i_dim]
                        = component_values_and_weight.first[i_dim];
            }
        }
        //------compute the edges of the reduced graph
        EdgeAttributeMap<T> border_edge_attribute_map = boost::get(boost::edge_bundle, this->reduced_graph);
        this->borders.clear();
        EdgeDescriptor edge_current, border_edge_current;
        uint32_t ind_border_edge = 0, comp1, comp2, component_source, component_target;
        VertexDescriptor<T> source_component, target_component;
        bool reducedEdgeExists;
        typename boost::graph_traits<Graph<T>>::edge_iterator ite_edg, ite_edg_end;
        for (boost::tie(ite_edg,ite_edg_end) = boost::edges(this->main_graph); ite_edg !=  ite_edg_end; ++ite_edg)
        {
            if (!edge_attribute_map(*ite_edg).realEdge)
            {   //edges linking the source or edge node do not take part
                continue;
            }
            edge_current = *ite_edg;
            //compute the connected components of the source and target of current_edge
            comp1       = vertex_attribute_map(boost::source(edge_current, this->main_graph)).in_component;
            comp2       = vertex_attribute_map(boost::target(edge_current, this->main_graph)).in_component;
            if (comp1==comp2)
            {   //this edge links two nodes in the same connected component
                continue;
            }
            //by convention we note component_source the smallest index and
            //component_target the largest
            component_source  = std::min(comp1,comp2);
            component_target  = std::max(comp1,comp2);
            //retrieve the corresponding vertex in the reduced graph
            source_component = boost::vertex(component_source, this->reduced_graph);
            target_component = boost::vertex(component_target, this->reduced_graph);
            //try to add the border-edge linking those components in the reduced graph
            boost::tie(border_edge_current, reducedEdgeExists)
                    = boost::edge(source_component, target_component, this->reduced_graph);
            if (!reducedEdgeExists)                
            {   //this border-edge did not already existed in the reduced graph
                //border_edge_current = boost::add_edge(source_component, target_component, this->reduced_graph).first;
                border_edge_current = boost::add_edge(source_component, target_component, this->reduced_graph).first;
                border_edge_attribute_map(border_edge_current).index  = ind_border_edge;
                border_edge_attribute_map(border_edge_current).weight = 0;
                ind_border_edge++;
                //create a new entry for the borders list containing this border
                this->borders.push_back(std::vector<EdgeDescriptor>(0));
            }
            //add the weight of the current edge to the weight of the border-edge
            border_edge_attribute_map(border_edge_current).weight += 0.5*edge_attribute_map(edge_current).weight;
            this->borders[border_edge_attribute_map(border_edge_current).index].push_back(edge_current);
        }
    }
    //=============================================================================================
    //================================          MERGE          ====================================
    //=============================================================================================
    uint32_t merge(bool is_cutoff)
    {
        // TODO: right now we only do one loop through the heap of potential mergeing, and only
        //authorize one mergeing per component. We could update the gain and merge until it is no longer
        //beneficial
        //check wether the energy can be decreased by removing edges from the reduced graph     
        //----load graph structure---
        VertexAttributeMap<T> vertex_attribute_map
                = boost::get(boost::vertex_bundle, this->main_graph);
        VertexAttributeMap<T> component_attribute_map
                = boost::get(boost::vertex_bundle, this->reduced_graph);
        EdgeAttributeMap<T> border_edge_attribute_map
                = boost::get(boost::edge_bundle, this->reduced_graph);
        EdgeAttributeMap<T> edge_attribute_map
                = boost::get(boost::edge_bundle, this->main_graph);
        VertexIndexMap<T> component_index_map = boost::get(boost::vertex_index, this->reduced_graph);
        //-----------------------------------
        EdgeDescriptor border_edge_current;
        typename boost::graph_traits<Graph<T>>::edge_iterator ite_border, ite_border_end;
        typename std::vector<EdgeDescriptor>::iterator ite_border_edge;
        VertexDescriptor<T> source_component, target_component;
        uint32_t ind_source_component, ind_target_component, border_edge_currentIndex;
        //gain_current is the vector of gains associated with each mergeing move
        //std::vector<T> gain_current(boost::num_edges(this->reduced_graph));
        //we store in merge_queue the potential mergeing with a priority on the potential gain
        std::priority_queue<ComponentsFusion<T>, std::vector<ComponentsFusion<T>>, lessComponentsFusion<T>> merge_queue;
        T gain; // the gain obtained by removing the border corresponding to the edge in the reduced graph
        for (boost::tie(ite_border,ite_border_end) = boost::edges(this->reduced_graph); ite_border !=  ite_border_end; ++ite_border)
        {
            //a first pass go through all the edges in the reduced graph and compute the gain obtained by
            //mergeing the corresponding vertices
            border_edge_current      = *ite_border;
            border_edge_currentIndex = border_edge_attribute_map(border_edge_current).index;
            //retrieve the two components corresponding to this border
            source_component = boost::source(border_edge_current, this->reduced_graph);
            target_component = boost::target(border_edge_current, this->reduced_graph);
            if (is_cutoff && component_attribute_map(source_component).weight >= this->parameter.cutoff
              &&component_attribute_map(target_component).weight >= this->parameter.cutoff)
            {
                continue;
            }
            ind_source_component = component_index_map(source_component);
            ind_target_component = component_index_map(target_component);
            //----now compute the gain of mergeing those two components-----
            // compute the fidelity lost by mergeing the two connected components
            std::pair<std::vector<T>, T> merge_gain = compute_merge_gain(source_component, target_component);
            // the second part is due to the removing of the border
            gain = merge_gain.second
                 + border_edge_attribute_map(border_edge_current).weight * this->parameter.reg_strenth;
            //mergeing_information store the indexes of the components as well as the edge index and the gain
            //in a structure ordered by the gain
            ComponentsFusion<T> mergeing_information(ind_source_component, ind_target_component, border_edge_currentIndex, gain);
            mergeing_information.merged_value = merge_gain.first;
            if (is_cutoff || gain>0)
            {   //it is beneficial to merge those two components
                //we add them to the merge_queue
                merge_queue.push(mergeing_information);
                //gain_current.at(border_edge_currentIndex) = gain;
            }
        }
        uint32_t n_merged = 0;
        //----go through the priority queue of merges and perform them as long as it is beneficial---
        //is_merged indicate which components no longer exists because they have been merged with a neighboring component
        std::vector<bool> is_merged(this->components.size(), false);
        //to_destroy indicates the components that are needed to be removed
        std::vector<bool> to_destroy(this->components.size(), false);
        while(merge_queue.size()>0)
        {   //loop through the potential mergeing and accept the ones that decrease the energy
            ComponentsFusion<T> mergeing_information = merge_queue.top();
            if (!is_cutoff && mergeing_information.merge_gain<=0)
            {   //no more mergeing provide a gain in energy
                break;
            }            
            merge_queue.pop();
            if (is_merged.at(mergeing_information.comp1) || (is_merged.at(mergeing_information.comp2)))
            {
                //at least one of the components have already been merged
                continue;
            }
            n_merged++;
            //---proceed with the fusion of comp1 and comp2----
            //add the vertices of comp2 to comp1
            this->components[mergeing_information.comp1].insert(this->components[mergeing_information.comp1].end()
                ,components[mergeing_information.comp2].begin(), this->components[mergeing_information.comp2].end());
            //if comp1 was saturated it might not be anymore
            this->saturated_components[mergeing_information.comp1] = false;
            //the new weight is the sum of both weights
            component_attribute_map(mergeing_information.comp1).weight
                           += component_attribute_map(mergeing_information.comp2).weight;
            //the new value is already computed in mergeing_information
            component_attribute_map(mergeing_information.comp1).value  = mergeing_information.merged_value;
            //we deactivate the border between comp1 and comp2
            for (ite_border_edge = this->borders.at(mergeing_information.border_index).begin();
                ite_border_edge != this->borders.at(mergeing_information.border_index).end() ; ++ite_border_edge)
            {
                 edge_attribute_map(*ite_border_edge).isActive = false;
            }
            is_merged.at(mergeing_information.comp1)  = true;
            is_merged.at(mergeing_information.comp2)  = true;
            to_destroy.at(mergeing_information.comp2) = true;
        }
        //we now rebuild the vectors components, rootComponents and saturated_components
        std::vector<std::vector<VertexDescriptor<T>>> new_components;
        std::vector<VertexDescriptor<T>> new_root_vertex;
        std::vector<bool> new_saturated_components;
        uint32_t ind_new_component = 0;
        for (uint32_t ind_com = 0; ind_com < this->components.size(); ind_com++)
        {
            if (to_destroy.at(ind_com))
            {   //this component has been removed
                continue;
            }//this components is kept
            new_components.push_back(this->components.at(ind_com));
            new_root_vertex.push_back(this->root_vertex.at(ind_com));
            new_saturated_components.push_back(saturated_components.at(ind_com));
            //if (is_merged.at(ind_com))
            //{   //we need to update the value of the vertex in this component
                for (uint32_t ind_ver = 0; ind_ver < this->components[ind_com].size(); ++ind_ver)
                {
                    vertex_attribute_map(this->components[ind_com][ind_ver]).value
                        = component_attribute_map(boost::vertex(ind_com, this->reduced_graph)).value;
                    vertex_attribute_map(this->components[ind_com][ind_ver]).in_component
                        = ind_new_component;//ind_com;
                }
            //}
            ind_new_component++;
        }
        this->components           = new_components;
        this->root_vertex          = new_root_vertex;
        this->saturated_components = new_saturated_components;
        return n_merged;
    }
    //=============================================================================================
    //================================          CUTOFF          ====================================
    //=============================================================================================
    void cutoff()
    {
        int i = 0;
        uint32_t n_merged;
        while (true)
        {
            //this->compute_connected_components();
            this->compute_reduced_graph();
            n_merged = merge(true);
            i++;
            if (n_merged==0 || i>50)
            {
                break;
            }
        }
    }
//    //=============================================================================================
//    //================================          CUTOFF          ====================================
//    //=============================================================================================
//    void cutoff()
//    {
//        // Loop through all components and merge the one smaller than the cutoff.
//        // It merges components which increase he energy the least
//        //----load graph structure---
//        VertexAttributeMap<T> vertex_attribute_map
//                = boost::get(boost::vertex_bundle, this->main_graph);
//        VertexAttributeMap<T> reduced_vertex_attribute_map
//                = boost::get(boost::vertex_bundle, this->reduced_graph);
//        EdgeAttributeMap<T> reduced_edge_attribute_map
//                = boost::get(boost::edge_bundle, this->reduced_graph);
//        EdgeAttributeMap<T> edge_attribute_map
//                = boost::get(boost::edge_bundle, this->main_graph);
//        VertexIndexMap<T> reduced_vertex_index_map = boost::get(boost::vertex_index, this->reduced_graph);
//        EdgeIndexMap<T> reduced_edge_index_map = get(&EdgeAttribute<T>::index, this->reduced_graph);
//        //-----------------------------------
//        typename boost::graph_traits<Graph<T>>::vertex_iterator ite_comp, ite_comp_end;
//        typename boost::graph_traits<Graph<T>>::out_edge_iterator ite_edg_out, ite_edg_out_end;
//        typename boost::graph_traits<Graph<T>>::in_edge_iterator ite_edg_in, ite_edg_in_end;
//        typename std::vector<EdgeDescriptor>::iterator ite_border_edge;
//        VertexDescriptor<T> current_vertex, neighbor_vertex;
//        //gain_current is the vector of gains associated with each mergeing move
//        //we store in merge_queue the potential mergeing with a priority on the potential gain
//        T gain; // the gain obtained by removing the border corresponding to the edge in the reduced graph
//        std::vector<bool> to_destroy(this->components.size(), false); //components merged to be removed
//        while (true)
//        {
//            this->compute_connected_components();
//            this->compute_reduced_graph();
//            bool has_merged = false;
//            std::cout << "CUTTING OFF : " << this->components.size() << "COMPONENTS " << std::endl;
//            for (boost::tie(ite_comp,ite_comp_end) = boost::vertices(this->reduced_graph); ite_comp !=  ite_comp_end; ++ite_comp)
//            {

//                current_vertex = *ite_comp;
//                if (reduced_vertex_attribute_map(current_vertex).weight > this->parameter.cutoff
//                   || to_destroy.at(reduced_vertex_index_map(current_vertex)))
//                {//component big enough to not be cut or already removed
//                    continue;
//                }
//                std::priority_queue<ComponentsFusion<T>, std::vector<ComponentsFusion<T>>, lessComponentsFusion<T>> merge_queue;
//std::cout << "COMPONENT " << reduced_vertex_index_map(current_vertex) << " IS OF SIZE"<< reduced_vertex_attribute_map(current_vertex).weight << std::endl;
//                for (boost::tie(ite_edg_out,ite_edg_out_end) = boost::out_edges(current_vertex, this->reduced_graph);
//                    ite_edg_out !=  ite_edg_out_end; ++ite_edg_out)
//                {   //explore all neighbors
//                     neighbor_vertex = boost::target(*ite_edg_out, this->reduced_graph);
//                     std::pair<std::vector<T>, T> merge_gain = compute_merge_gain(current_vertex, neighbor_vertex);
//                     gain = merge_gain.second
//                          + reduced_edge_attribute_map(*ite_edg_out).weight * this->parameter.reg_strenth;
//                     ComponentsFusion<T> mergeing_information(reduced_vertex_index_map(current_vertex), reduced_vertex_index_map(neighbor_vertex)
//                                       , reduced_edge_index_map(*ite_edg_out), gain);
//                     mergeing_information.merged_value = merge_gain.first;
//                     merge_queue.push(mergeing_information);
//std::cout << "         NEI OUT " <<reduced_vertex_index_map(neighbor_vertex)  << " GAIN"<< gain << std::endl;

//                }
//                for (boost::tie(ite_edg_in,ite_edg_in_end) = boost::in_edges(current_vertex, this->reduced_graph);
//                    ite_edg_in !=  ite_edg_in_end; ++ite_edg_in)
//                {   //explore all neighbors
//                     neighbor_vertex = boost::source(*ite_edg_in, this->reduced_graph);
//                     std::pair<std::vector<T>, T> merge_gain = compute_merge_gain(current_vertex, neighbor_vertex);
//                     gain = merge_gain.second
//                          + reduced_edge_attribute_map(*ite_edg_in).weight * this->parameter.reg_strenth;
//                     ComponentsFusion<T> mergeing_information(reduced_vertex_index_map(current_vertex), reduced_vertex_index_map(neighbor_vertex)
//                                       , reduced_edge_index_map(*ite_edg_in), gain);
//                     mergeing_information.merged_value = merge_gain.first;
//                     merge_queue.push(mergeing_information);
//std::cout << "         NEI IN" <<reduced_vertex_index_map(neighbor_vertex)  << " GAIN"<< gain << std::endl;

//                }
//                if (merge_queue.empty())
//                {
//                    continue;
//                }
//                has_merged = true;
//                //select the most advantegeous neighboring components and merge it.
//                ComponentsFusion<T> mergeing_information = merge_queue.top();
//std::cout << "BEST NEIGHBORS = " << mergeing_information.comp1 << " - " << mergeing_information.comp2 << " = " << mergeing_information .merge_gain
//          << "   Weight " << reduced_vertex_attribute_map(mergeing_information.comp2).weight << std::endl;
//                this->components[mergeing_information.comp1].insert(this->components[mergeing_information.comp1].end()
//                    ,components[mergeing_information.comp2].begin(), this->components[mergeing_information.comp2].end());
//                //the new weight is the sum of both weights
//                reduced_vertex_attribute_map(mergeing_information.comp1).weight
//                               += reduced_vertex_attribute_map(mergeing_information.comp2).weight;
//                //the new value is already computed in mergeing_information
//                reduced_vertex_attribute_map(mergeing_information.comp1).value  = mergeing_information.merged_value;
//                //we deactivate the border between comp1 and comp2
//                for (ite_border_edge = this->borders.at(mergeing_information.border_index).begin();
//                    ite_border_edge != this->borders.at(mergeing_information.border_index).end() ; ++ite_border_edge)
//                {
//                     edge_attribute_map(*ite_border_edge).isActive = false;
//                }
//                to_destroy.at(mergeing_information.comp2) = true;
//std::cout << "=> " << reduced_vertex_index_map(current_vertex) << " IS OF SIZE"<< reduced_vertex_attribute_map(current_vertex).weight << std::endl;

//            }
//            //if (!has_merged)
//            //{
//                break;
//            //}
//        }
//        //we now rebuild the vectors components, rootComponents and saturated_components
//        std::vector<std::vector<VertexDescriptor<T>>> new_components;
//        uint32_t ind_new_component = 0;
//        for (uint32_t ind_com = 0; ind_com < this->components.size(); ind_com++)
//        {
//            if (to_destroy.at(ind_com))
//            {   //this component has been removed
//                continue;
//            }//this components is kept
//            new_components.push_back(this->components.at(ind_com));
//            //if (is_merged.at(ind_com))
//            //{   //we need to update the value of the vertex in this component
//                for (uint32_t ind_ver = 0; ind_ver < this->components[ind_com].size(); ++ind_ver)
//                {
//                    vertex_attribute_map(this->components[ind_com][ind_ver]).value
//                        = reduced_vertex_attribute_map(boost::vertex(ind_com, this->reduced_graph)).value;
//                    vertex_attribute_map(this->components[ind_com][ind_ver]).in_component
//                        = ind_new_component;//ind_com;
//                }
//            //}
//            ind_new_component++;
//        }
//        this->components           = new_components;
//    }
//===============================================================================================
//==========================saturateComponent====================================================
//===============================================================================================
    inline void saturateComponent(const uint32_t & ind_com)
    {   //this component is uncuttable and needs to be removed from further graph-cuts
        EdgeAttributeMap<T> edge_attribute_map
                                    = boost::get(boost::edge_bundle, this->main_graph);
        this->saturated_components[ind_com] = true;
        for (uint32_t i_ver = 0;  i_ver < this->components[ind_com].size(); i_ver++)
        {
            VertexDescriptor<T> desc_v = this->components[ind_com][i_ver];
            // because of the adjacency structure NEVER access edge (source,v) directly!
            EdgeDescriptor edg_ver2source = boost::edge(desc_v, this->source,this->main_graph).first;
            EdgeDescriptor edg_source2ver = edge_attribute_map(edg_ver2source).edge_reverse; //use edge_reverse instead
            EdgeDescriptor edg_sink2ver   = boost::edge(desc_v, this->sink,this->main_graph).first;
            // we set the capacities of edges to source and sink to zero
            edge_attribute_map(edg_source2ver).capacity = 0.;
            edge_attribute_map(edg_sink2ver).capacity = 0.;
        }
    }
};
}
