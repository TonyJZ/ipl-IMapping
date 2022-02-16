#pragma once
#include <core/iplcore.h>
#include <optimization/LabelOptimization.h>

//boost
#include <boost/unordered_map.hpp>
#include <pcl/segmentation/boost.h>

namespace ipl
{
	//use graph cut to solve the indoor layout optimization
	class IPL_BASE_API IndoorLayoutOptimizationByGC : public LabelOptimization
	{
	public:
		typedef boost::adjacency_list_traits< boost::vecS, boost::vecS, boost::directedS > Traits;

		typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::directedS,
			boost::property< boost::vertex_name_t, std::string,
			boost::property< boost::vertex_index_t, long,
			boost::property< boost::vertex_color_t, boost::default_color_type,
			boost::property< boost::vertex_distance_t, long,
			boost::property< boost::vertex_predecessor_t, Traits::edge_descriptor > > > > >,
			boost::property< boost::edge_capacity_t, double,
			boost::property< boost::edge_residual_capacity_t, double,
			boost::property< boost::edge_reverse_t, Traits::edge_descriptor > > > > mGraph;

		typedef boost::property_map< mGraph, boost::edge_capacity_t >::type CapacityMap;

		typedef boost::property_map< mGraph, boost::edge_reverse_t>::type ReverseEdgeMap;

		typedef Traits::vertex_descriptor VertexDescriptor;

		typedef boost::graph_traits< mGraph >::edge_descriptor EdgeDescriptor;

		typedef boost::graph_traits< mGraph >::out_edge_iterator OutEdgeIterator;

		typedef boost::graph_traits< mGraph >::vertex_iterator VertexIterator;

		typedef boost::property_map< mGraph, boost::edge_residual_capacity_t >::type ResidualCapacityMap;

		typedef boost::property_map< mGraph, boost::vertex_index_t >::type IndexMap;

		typedef boost::graph_traits< mGraph >::in_edge_iterator InEdgeIterator;

	public:
		IndoorLayoutOptimizationByGC() :
			epsilon_(0.0001)
		{

		}

		IndoorLayoutOptimizationByGC(double epsilon) :
			epsilon_(epsilon)
		{

		}

		virtual ~IndoorLayoutOptimizationByGC()
		{

		}

		
		virtual std::vector<std::vector<int> > getLabels()
		{
			return clusters_;
		}

		virtual bool beginBuildGraph(int vextex_num);

		virtual bool endBuildGraph();

		virtual bool
			add_UnaryCost(int key, double source_weight, double sink_weight);

		virtual bool
			add_UndirectedBinaryCost(int source_key, int target_key, double edge_weight);

		virtual bool
			add_DirectedBinaryCost(int source_key, int target_key, double edge_weight);
// 
// 		virtual bool
// 			addEdge(int source_key, int target_key, double weight);

	protected:
		bool
			addEdge(int source, int target, double weight);

		void
			assembleLabels(ResidualCapacityMap& residual_capacity);

	private:
		/////////////////////////graph cut  /////////////////////////
		/** \brief Stores the graph for finding the maximum flow. */
		boost::shared_ptr<mGraph> graph_;

		/** \brief Stores the capacity of every edge in the graph. */
		boost::shared_ptr<CapacityMap> capacity_;

		/** \brief Stores reverse edges for every edge in the graph. */
		boost::shared_ptr<ReverseEdgeMap> reverse_edges_;

		/** \brief Stores the vertices of the graph. */
		std::vector< VertexDescriptor > vertices_;

		/** \brief Stores the information about the edges that were added to the graph. It is used to avoid the duplicate edges. */
		std::vector< std::set<int> > edge_marker_;

		/** \brief Stores the vertex that serves as source. */
		VertexDescriptor source_;

		/** \brief Stores the vertex that serves as sink. */
		VertexDescriptor sink_;

		/** \brief Stores the maximum flow value that was calculated during the segmentation. */
		double max_flow_;

		/** \brief Used for comparison of the floating point numbers. */
		double epsilon_;

		int vextex_num_;   //¶¥µã×ÜÊý

		std::vector <std::vector<int> > clusters_;
	};
}

