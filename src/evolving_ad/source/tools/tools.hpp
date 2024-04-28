// class WatchDog
// {
//   public:
//     WatchDog() = delete;
//     WatchDog(ros::NodeHandle &nh, const double time_period = 1.0, const double time_max_delay = 10.0);
//     ~WatchDog() = default;
//     void FeedDog();
//     bool GetTimeOutStatus();

//   private:
//     void TimerCallback(const ros::TimerEvent &event);
//     ros::Timer timer_;

//     double time_accumulate_ = 0;
//     double time_period_ = 0;
//     double time_max_delay_ = 0;

//     bool time_out_flag_ = false;
// };