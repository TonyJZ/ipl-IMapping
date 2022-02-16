#include <optimization/IndoorLayoutOptimizationByGC.h>

bool
ipl::IndoorLayoutOptimizationByGC::beginBuildGraph(int vextex_num)
{
	// 	int number_of_points = static_cast<int> (input_->points.size ());
	// 	int number_of_indices = static_cast<int> (indices_->size ());
	// 
	// 	if (input_->points.size () == 0 || number_of_points == 0 || foreground_points_.empty () == true )
	// 		return (false);

	if (vextex_num <= 0)
		return false;

	vextex_num_ = vextex_num;

	graph_.reset();
	graph_ = boost::shared_ptr< mGraph >(new mGraph());

	capacity_.reset();
	capacity_ = boost::shared_ptr<CapacityMap>(new CapacityMap());
	*capacity_ = boost::get(boost::edge_capacity, *graph_);

	reverse_edges_.reset();
	reverse_edges_ = boost::shared_ptr<ReverseEdgeMap>(new ReverseEdgeMap());
	*reverse_edges_ = boost::get(boost::edge_reverse, *graph_);

	VertexDescriptor vertex_descriptor(0);
	vertices_.clear();
	vertices_.resize(vextex_num_ + 2, vertex_descriptor);

	std::set<int> out_edges_marker;
	edge_marker_.clear();
	edge_marker_.resize(vextex_num_ + 2, out_edges_marker);

	for (int i_point = 0; i_point < vextex_num_ + 2; i_point++)
		vertices_[i_point] = boost::add_vertex(*graph_);

	source_ = vertices_[vextex_num_];
	sink_ = vertices_[vextex_num_ + 1];

	//cor_seed_id.indices.clear();
	//cor_seed_id.indices.resize(number_of_indices, -1);
	// 	if(cor_seed_id)
	// 	{
	// 		delete[] cor_seed_id;
	// 		cor_seed_id=NULL;
	// 	}

	//	cor_seed_id=new int[number_of_indices];

	return (true);
}

bool
ipl::IndoorLayoutOptimizationByGC::add_UnaryCost(int key, double source_weight, double sink_weight)
{
	
	addEdge(static_cast<int> (source_), key, source_weight);
	addEdge(key, static_cast<int> (sink_), sink_weight);

	return (true);
}

bool 
ipl::IndoorLayoutOptimizationByGC::add_UndirectedBinaryCost(int source_key, int target_key, double edge_weight)
{
	addEdge(source_key, target_key, edge_weight);
	addEdge(target_key, source_key, edge_weight);

	return (true);
}

bool
ipl::IndoorLayoutOptimizationByGC::add_DirectedBinaryCost(int source_key, int target_key, double edge_weight)
{
	addEdge(source_key, target_key, edge_weight);

	return (true);
}

bool
ipl::IndoorLayoutOptimizationByGC::addEdge(int source, int target, double weight)
{
	std::set<int>::iterator iter_out = edge_marker_[source].find(target);
	if (iter_out != edge_marker_[source].end())
		return (false);

	EdgeDescriptor edge;
	EdgeDescriptor reverse_edge;
	bool edge_was_added, reverse_edge_was_added;

	boost::tie(edge, edge_was_added) = boost::add_edge(vertices_[source], vertices_[target], *graph_);
	boost::tie(reverse_edge, reverse_edge_was_added) = boost::add_edge(vertices_[target], vertices_[source], *graph_);
	if (!edge_was_added || !reverse_edge_was_added)
		return (false);

	(*capacity_)[edge] = weight;
	(*capacity_)[reverse_edge] = 0.0;
	(*reverse_edges_)[edge] = reverse_edge;
	(*reverse_edges_)[reverse_edge] = edge;
	edge_marker_[source].insert(target);

	return (true);
}

bool
ipl::IndoorLayoutOptimizationByGC::endBuildGraph()
{
	clusters_.clear();
	
	ResidualCapacityMap residual_capacity = boost::get(boost::edge_residual_capacity, *graph_);

	max_flow_ = boost::boykov_kolmogorov_max_flow(*graph_, source_, sink_);

	std::cout << "the total flow is " << max_flow_ << std::endl;

	assembleLabels(residual_capacity);

	// 	clusters.reserve (clusters_.size ());
	// 	std::copy (clusters_.begin (), clusters_.end (), std::back_inserter (clusters));

	return (true);
}

void
ipl::IndoorLayoutOptimizationByGC::assembleLabels(ResidualCapacityMap& residual_capacity)
{
	// 	std::vector<int> labels;
	// 	labels.resize (input_->points.size (), 0);
	// 	int number_of_indices = static_cast<int> (indices_->size ());
	// 	for (int i_point = 0; i_point < number_of_indices; i_point++)
	// 		labels[(*indices_)[i_point]] = 1;

	clusters_.clear();

	std::vector<int> segment;
	clusters_.resize(2, segment);

	std::vector<double> res;
	res.resize(vextex_num_, 0);

	OutEdgeIterator edge_iter, edge_end;
	int i = 0;
	for (boost::tie(edge_iter, edge_end) = boost::out_edges(source_, *graph_); edge_iter != edge_end; edge_iter++)
	{
		int vId = static_cast<int> (edge_iter->m_target);

		res[static_cast<int> (edge_iter->m_target)] = residual_capacity[*edge_iter];

		if (residual_capacity[*edge_iter] > epsilon_)
		{//source
			clusters_[0].push_back(vId);
		}
		else
		{//sink
			clusters_[1].push_back(vId);
		}

		i++;
	}
}

