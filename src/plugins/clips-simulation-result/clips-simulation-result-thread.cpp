#include "clips-simulation-result-thread.h"

// #include <clipsmm.h>
#include <iostream>
#include <fstream>
#include <variant>

using namespace fawkes;

/** Constructor. */
ClipsSimulationResultThread::ClipsSimulationResultThread()
: Thread("ClipsSimulationResultThread", Thread::OPMODE_WAITFORWAKEUP),
  CLIPSFeature("simulation-result"),
  CLIPSFeatureAspect(this)
{
}

/** Destructor. */
ClipsSimulationResultThread::~ClipsSimulationResultThread()
{
}

void
ClipsSimulationResultThread::init()
{   
    // written_    = false;
    // replay_     = config->get_string("/gazsim/llsf-statistics/log");
}

void
ClipsSimulationResultThread::finalize()
{
    envs_.clear();
}

void
ClipsSimulationResultThread::clips_context_init(const std::string           &env_name,
                                                LockPtr<CLIPS::Environment> &clips)
{
    envs_[env_name] = clips;
    logger->log_info(name(), "rabbit!!Called to initialize environment %s", env_name.c_str());
    clips.lock();
    
    clips->add_function(
        "save-chosen-orders-with-max-points",
        sigc::slot<void, CLIPS::Values>(sigc::bind<0>(
            sigc::mem_fun(*this, &ClipsSimulationResultThread::get_list_of_chosen_orders_with_max_points), env_name)));

    clips->add_function(
        "simulation-result-into-csv",
        sigc::slot<void, CLIPS::Values>(sigc::bind<0>(
            sigc::mem_fun(*this, &ClipsSimulationResultThread::clips_simulation_result_into_csv), env_name)));
    
    clips.unlock();

}

void
ClipsSimulationResultThread::clips_context_destroyed(const std::string &env_name)
{
    envs_.erase(env_name);
	logger->log_info(name(), "Removing environment %s", env_name.c_str());
}

void
ClipsSimulationResultThread::get_list_of_chosen_orders_with_max_points(const std::string &env_name,
                                                        CLIPS::Values      values) 
{
    std::string order_ID;
    long long int val;
    for (const auto& value : values) {
        if (value.type() == CLIPS::TYPE_SYMBOL || value.type() == CLIPS::TYPE_STRING){
            order_ID = value.as_string();
            // std::get<0>(flag) = val;
            chosen_orders.push_back(order_ID);
            logger->log_info(name(), "hahaha %s", order_ID.c_str());
        }
        else if (value.type() == CLIPS::TYPE_INTEGER){
            val = value.as_integer();
            // std::get<1>(flag) = val;
            logger->log_info(name(), "hahaha %d", val);
        }
        else{
            logger->log_info(name(), "hahaha");
        }
    }
    max_points_for_chosen_orders[order_ID] = val;
    for (const auto& item:chosen_orders){
        logger->log_info(name(), "Chosen:order %s", item.c_str());
    }
    for (const auto& entry : max_points_for_chosen_orders) {
        int points = entry.second;
        std::string order_ID = entry.first;
        // std::tie(order_ID, points) = entry;
        logger->log_info(name(), "Order: %s with max points %d", order_ID.c_str(), points);
    }
}

std::map<std::string, std::vector<ClipsSimulationResultThread::TableRow>> 
ClipsSimulationResultThread::find_scored_points_in_refbox_debug_log(const std::string &env_name)

{
    // the structure of tableData is {order: (points, reason)}
    std::map<std::string, std::vector<TableRow>> tableData;
    std::ifstream logFile("/home/xliu/fawkes-robotino/bin/refbox-debug_latest.log");
    std::string line;
    bool process_following_lines = false;
    bool skip_next_line = false;
    
    if (logFile.is_open()) {
        while (getline(logFile, line)) {
            if (skip_next_line) {
                skip_next_line = false;
                continue;
            }
            if (process_following_lines) {
                if (line.find("|") != std::string::npos &&
                    line.find("path") != std::string::npos &&
                    line.find("value") != std::string::npos &&
                    line.find("list-value") != std::string::npos) {
                
                    break;
                }
                std::istringstream iss(line);
                std::string token;
                std::vector<std::string> tokens;
                while (getline(iss, token, '|')) {  // Use '|' as delimiter
                    tokens.push_back(token);
                }
                if (tokens.size() >= 5) {
                    // Trim and convert tokens
                    std::string pointsAtSecond = tokens[1];
                    std::string orderAtFourth = tokens[3];
                    std::string reasonAtSeventh = tokens[6];

                    // Remove leading and trailing whitespaces
                    auto trim = [](std::string& str) {
                        str.erase(str.begin(), std::find_if(str.begin(), str.end(), [](unsigned char ch) {
                            return !std::isspace(ch);
                        }));
                        str.erase(std::find_if(str.rbegin(), str.rend(), [](unsigned char ch) {
                            return !std::isspace(ch);
                        }).base(), str.end());
                    };

                    trim(pointsAtSecond);
                    trim(orderAtFourth);
                    trim(reasonAtSeventh);
                    // Convert to appropriate types
                    int points = std::stoi(pointsAtSecond);
                    TableRow newRow = std::make_tuple(points, reasonAtSeventh);
                    // Store in the map
                    if (tableData.find(orderAtFourth) != tableData.end()){
                        tableData[orderAtFourth].push_back(newRow);
                    } else {
                        tableData[orderAtFourth] = std::vector<TableRow>{newRow};
                    }
                }
                continue;
            }
            if (line.find("|") != std::string::npos &&
                line.find("points") != std::string::npos && 
                line.find("team") != std::string::npos && 
                line.find("order") != std::string::npos && 
                line.find("game-time") != std::string::npos &&
                line.find("phase") != std::string::npos && 
                line.find("reason") != std::string::npos) {
                process_following_lines = true;
                skip_next_line = true;
            }
        }
        logFile.close();
    } else {
        logger->log_info(name(), "test-------------game over over over %s", env_name.c_str());
    }
    // for (const auto& entry : tableData) {
    //     logger->log_info(name(), "Order: %s", entry.first.c_str());
    //     for (const auto& subentry : entry.second) {
    //         int points;
    //         std::string reason;
    //         std::tie(points, reason) = subentry; // Unpack the tuple
    //         logger->log_info(name(), "%d : %s", points, reason.c_str()); 
    //     }
    // }
    return tableData;
}

std::map<std::string, int>
ClipsSimulationResultThread::imply_scored_points_for_chosen_orders(int overall_points, 
                                    std::map<std::string, std::vector<TableRow>> tableData, 
                                    const std::string& csv_file_path)
{
    // To verify if total points from tableData equals overall_points
    int total = 0;
    for (const auto& pair : tableData) {
        for (const auto& row : pair.second) {
            int value;
            std::string dummy; 
            std::tie(value, dummy) = row;
            total += value;
        }
    }
    logger->log_info(name(), "Compare overall points %d %d", total, overall_points);

    std::map<std::string, int> scored_points_of_chosen_orders;
    for (auto& elem : chosen_orders) {
        // capital character o 
        std::string element = elem;
        if (element[0] == 'O') {
            element.erase(0, 1);  // Remove the first character if it is '0'
        }
        // logger->log_info(name(), "bulabula: %s", element.c_str());   
        if (tableData.find(element) != tableData.end()) {
            logger->log_info(name(), "exists %s", element.c_str());
            int calculate_points = 0;
            for (const auto& item: tableData.find(element)->second){
                int points;
                std::string reason;
                std::tie(points, reason) = item;
                logger->log_info(name(), "step points %d with reason %s", points, reason.c_str());
                calculate_points += points;
            }
            scored_points_of_chosen_orders[elem] = calculate_points;
        } else {
            scored_points_of_chosen_orders[elem] = 0;
            logger->log_info(name(), "non exists %s", element.c_str());
        }
        logger->log_info(name(), "--------------");
    } 
    return scored_points_of_chosen_orders;
}  

void 
ClipsSimulationResultThread::label_data_in_csv_with_ratio_of_scored_points(
                    std::map<std::string, float> ratio_of_scored_points_for_chosen_orders,
                                                const std::string& csv_file_path, int n) 
                        
{   
    // read csv file, find rows of current game, 
    // find the chosen order for each row, label in the line end
    std::ifstream file(csv_file_path, std::ios::ate | std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file\n";
        throw std::runtime_error("Failed to open file.");
    }
    auto end_pos = file.tellg();
    std::string line;
    // To store the position of each line start
    std::vector<std::streampos> line_positions;

    // Seek the positions of line starts from the end
    for (auto pos = end_pos-1; pos > 0; ) {
        file.seekg(pos);
        char ch;
        file.get(ch);
        if (ch == '\n') {
            if (pos != end_pos - 1) {
                line_positions.push_back(file.tellg());
            }
            if (line_positions.size() >= n) break;
        }
        pos -= 1;
    }
    std::reverse(line_positions.begin(), line_positions.end());
    // // Find and print the lines according to line_positions
    // for (auto pos : line_positions) {
    //     file.clear();
    //     file.seekg(pos);
    //     std::getline(file, line);
    //     // Split the line by the delimiter ','
    //     std::istringstream lineStream(line);
    //     std::string token;
    //     int count = 0;
    //     std::string fifthElement;

    //     // Extract tokens up to the fifth element, it is the Order ID
    //     while (std::getline(lineStream, token, ',') && count < 5) {
    //         count++;
    //         if (count == 5) {
    //             fifthElement = token;
    //         }
    //     }

    //     if (count >= 5) {
    //         logger->log_info(name(), "FileReader %s", fifthElement.c_str());
    //     } else if (!line.empty()){
    //         logger->log_info(name(), "FileReader %s", line.c_str());
    //     }else {
    //         logger->log_info(name(), "FileReader Empty");
    //     }
    // }

    std::streampos modify_start_pos = line_positions.size() >= n ? line_positions[line_positions.size() - n] : std::streampos(0);
    
    // Rewind to start of file for reading
    file.clear();
    file.seekg(0);
    
    std::ofstream file2_stream("/home/xliu/fawkes-robotino/bin/example_liu_label_with_score_ratio.csv");
    if (!file2_stream.is_open()) {
        std::cerr << "Failed to open output file\n";
        throw std::runtime_error("Failed to open output file.");
    }
    // Read and write each line, modifying last n lines
    while (std::getline(file, line)) {
        if (file.tellg() > modify_start_pos && !line.empty()) {
            std::stringstream ss(line);
            std::string item;
            std::vector<std::string> tokens;
            while (getline(ss, item, ',')) {
                tokens.push_back(item);
            }
            if (tokens.size() >= 5) {
                line += std::to_string(ratio_of_scored_points_for_chosen_orders[tokens[4]]);
                // logger->log_info(name(), "Label Label 3 %s", tokens[4].c_str());
            }
            file2_stream << line << std::endl;
        }      
    }
    file.close();
    file2_stream.close();
}                                                            

void 
ClipsSimulationResultThread::clips_simulation_result_into_csv(const std::string &env_name, 
                                                             CLIPS::Values      values) 
{
    // get all printout data through function parameters
    // size_t values_length = values.size();
    // for (size_t i = 0; i < values_length; i++) {
    //     std::cout << "Parameters coming: " << values[i] << std::endl;
    // }
    logger->log_info(name(), "Successfully lala invoke self-defined function inside environment %s", env_name.c_str());
    std::string csv_file_path = "/home/xliu/fawkes-robotino/bin/example_liu.csv";
    // ratio_of_scored_points_for_chosen_orders
    // label_data_in_csv_with_ratio_of_scored_points(csv_file_path, 9);
    // std::string chosen_order;
    // if (values.size() > 4){
    //     chosen_order = values[4].as_string();
    //     // logger->log_info(name(), "bulabula %s", chosen_order.c_str());
    // }
    if ((values[2].type() == CLIPS::TYPE_SYMBOL || values[2].type() == CLIPS::TYPE_STRING) 
        && values[2].as_string() == "Overall"){

        auto tableData = find_scored_points_in_refbox_debug_log(env_name); 
        for (const auto& entry : tableData) {
            for (const auto& subentry : entry.second) {
                int points;
                std::string reason;
                std::tie(points, reason) = subentry; // Unpack the tuple
            }
        }

        if (!tableData.empty()) {
            // invoke function to imply points scored by every chosen order
            auto scored_points_of_chosen_orders = imply_scored_points_for_chosen_orders(values[3].as_integer(), tableData, csv_file_path);
            for (const auto& pair:scored_points_of_chosen_orders){
                logger->log_info(name(), "Scoring !! %s : %d", pair.first.c_str(), pair.second);
            }
            for (const auto& order:chosen_orders){
                if (max_points_for_chosen_orders.find(order) == max_points_for_chosen_orders.end() ||
                    scored_points_of_chosen_orders.find(order) == scored_points_of_chosen_orders.end()){
                    logger->log_info(name(), "Error!"); 
                }
                else{
                    auto scored = scored_points_of_chosen_orders.find(order)->second;
                    auto overall = max_points_for_chosen_orders.find(order)->second;
                    float ratio = static_cast<float>(scored) / overall;
                    ratio_of_scored_points_for_chosen_orders[order] = ratio;
                    logger->log_info(name(), "Order %s: %d / %d = Ratio: %f", order.c_str(), scored, overall, ratio);
                }
            }
            // invoke function to label .csv file with "ratio_of_scored_points_for_chosen_orders"
            label_data_in_csv_with_ratio_of_scored_points(ratio_of_scored_points_for_chosen_orders,
                                                        csv_file_path, 9);
        } else {
            logger->log_info(name(), "Couldn't find the table of scored points!"); 
        }
    }
    // write into a csv file
    std::ofstream myfileliu;
    myfileliu.open (csv_file_path, std::ios_base::app);
    if (!myfileliu.is_open()) {
        std::cerr << "Failed to open file: " << std::endl;
        logger->log_info(name(), "wrong-------------wrong %s", env_name.c_str());
        return;
    } 
    for (const auto& value : values) {
        logger->log_info(name(), "test-------------inside loop %s", env_name.c_str());

        if (value.type() == CLIPS::TYPE_SYMBOL || value.type() == CLIPS::TYPE_STRING){
            std::string val = value.as_string();
            myfileliu << val << ","; // Writing each value followed by a comma
            logger->log_info(name(), "test-------------string or symbol %s", env_name.c_str());
        }
        else if (value.type() == CLIPS::TYPE_INTEGER){
            long long int val = value.as_integer();
            myfileliu << val << ","; // Writing each value followed by a comma
            logger->log_info(name(), "test-------------integer %s", env_name.c_str());
        }
        else if (value.type() == CLIPS::TYPE_FLOAT) {
            double val = value.as_float();
            myfileliu << val << ","; // Writing each value followed by a comma
            logger->log_info(name(), "test-------------float %s", env_name.c_str());
        }
        else {
            logger->log_info(name(), "type-------------other type %s", env_name.c_str());
        }
    }
    myfileliu << std::endl;
    logger->log_info(name(), "test-------------end %s", env_name.c_str());
    myfileliu.close();
}                                                           

void
ClipsSimulationResultThread::loop()
{
}