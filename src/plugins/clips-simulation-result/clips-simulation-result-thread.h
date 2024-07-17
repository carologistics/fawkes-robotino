#ifndef _PLUGINS_CLIPS_SIMULATION_RESULT_CLIPS_SIMULATION_RESULT_THREAD_H_
#define _PLUGINS_CLIPS_SIMULATION_RESULT_CLIPS_SIMULATION_RESULT_THREAD_H_

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <plugins/clips/aspect/clips_feature.h>

#include <map>
#include <string>
#include <vector>
#include <clipsmm.h>

class ClipsSimulationResultThread : public fawkes::Thread,
									public fawkes::LoggingAspect,
                            		public fawkes::CLIPSFeature,
                            		public fawkes::CLIPSFeatureAspect
{
public:
	ClipsSimulationResultThread();
	virtual ~ClipsSimulationResultThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	// for CLIPSFeature
	virtual void clips_context_init(const std::string                   &env_name,
	                                fawkes::LockPtr<CLIPS::Environment> &clips);
	virtual void clips_context_destroyed(const std::string &env_name);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	using TableRow = std::tuple<int, std::string>;
	std::map<std::string, std::vector<TableRow>> find_scored_points_in_refbox_debug_log(const std::string &env_name);

	std::vector<std::string> chosen_orders;
	std::map<std::string, int> max_points_for_chosen_orders;
	std::map<std::string, float> ratio_of_scored_points_for_chosen_orders;
	void get_list_of_chosen_orders_with_max_points(const std::string &env_name, CLIPS::Values values);
	std::map<std::string, int> imply_scored_points_for_chosen_orders(int overall_points, std::map<std::string, std::vector<TableRow>> tableData, const std::string& csv_file_path);
	// 
	void label_data_in_csv_with_ratio_of_scored_points(std::map<std::string, float> ratio_of_scored_points_for_chosen_orders,const std::string& csv_file_path, int n);
	void clips_simulation_result_into_csv(const std::string &env_name, CLIPS::Values values);
private:
	std::map<std::string, fawkes::LockPtr<CLIPS::Environment>> envs_;
};

#endif