#ifndef event_filter_H_
#define event_filter_H_

#include <ros/ros.h>
namespace event_filter
{
	void median_filter();
	void parse_inputs();
	void hUseIPPL();
	void hPadImage();
}

#endif // event_filter_H_