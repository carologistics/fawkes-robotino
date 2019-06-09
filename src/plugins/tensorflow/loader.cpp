#include "loader.h"

TF_Plugin_Loader::TF_Plugin_Loader(std::string name, fawkes::Logger *logger)
    : name_(name), logger_(logger) {
  this->logger_->log_debug(name_.c_str(), "New TF_Plugin_Loader loaded");
}

TF_Plugin_Loader::~TF_Plugin_Loader() {
  this->logger_->log_debug(name_.c_str(), "TF_Plugin_Loader destroyed");
}
