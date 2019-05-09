#ifndef J_PCL_FILTERS_BILATTERAL_H_
#define J_PCL_FILTERS_BILATTERAL_H_
#include<pcl/filters/filter.h>
#include<pcl/kdtree/kdtree.h>
namespace pcl {
	template<typename PointT>
	class BilateralFilter :public Filter<PointT> {
		using Filter<PointT>::input_;
		typedef typename Filter<PointT>::PointCloud PointCloud;
		typedef typename pcl::KdTree<PointT>::Ptr KdTreePtr;

	public:
		BilateralFilter() :sigma_r_(std::numeric_limits<double>::max()) {

		}
		void  setSigmaS(const double sigma_s) {
			sigma_s_=sigma_s
		}
		double getSigmaS() {
			return sigma_s_;
		}

		void  setSigmaR(const double sigma_r) {
			sigma_r_ = sigma_r
		}
		double getSigmaR() {
			return sigma_r_;
		}

	private:
		double sigma_s_;
		double sigma_r_;
	};

	
	};
}
#endif // !J_PCL_FILTERS_BILATTERAL_H_
