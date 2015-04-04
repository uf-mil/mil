#include <cstdlib>
#include <stdexcept>
#include <sstream>

#include <boost/function.hpp>
#include <boost/utility.hpp>

#include "libusb.h"

namespace libusbcpp {


class Error : public std::runtime_error {
public:
    explicit Error(std::string const & what_arg) :
        std::runtime_error(what_arg) {
    }
};

int check_error(int res) {
    if(res >= 0) return res;
    throw Error(
        std::string(libusb_error_name(static_cast<libusb_error>(res))) + " " +
        std::string(libusb_strerror  (static_cast<libusb_error>(res))));
}

class Device : boost::noncopyable {
public:
    libusb_device_handle * const handle_;
    Device(libusb_device_handle * handle) :
        handle_(handle) {
    }
    ~Device() {
        libusb_close(handle_);
    }
    
    bool kernel_driver_active(int interface_number) {
        return check_error(libusb_kernel_driver_active(handle_, interface_number)) == 1;
    }
    void detach_kernel_driver(int interface_number) {
        check_error(libusb_detach_kernel_driver(handle_, interface_number));
    }
    
    int get_configuration() {
        int res;
        check_error(libusb_get_configuration(handle_, &res));
        return res;
    }
    void set_configuration(int configuration) {
        check_error(libusb_set_configuration(handle_, configuration));
    }
    
    void claim_interface(int interface_number) {
        check_error(libusb_claim_interface(handle_, interface_number));
    }
    void release_interface(int interface_number) {
        check_error(libusb_release_interface(handle_, interface_number));
    }
    void set_interface_alt_setting(int interface_number, int alternate_setting) {
        check_error(libusb_set_interface_alt_setting(handle_, interface_number, alternate_setting));
    }
    
    size_t control_transfer(uint8_t bmRequestType, uint8_t bRequest,
    uint16_t wValue, uint16_t wIndex, unsigned char * data, uint16_t wLength,
    unsigned int timeout=0) {
        return check_error(libusb_control_transfer(handle_, bmRequestType,
            bRequest, wValue, wIndex, data, wLength, timeout));
    }
    
    void interrupt_transfer(unsigned char endpoint, unsigned char * data,
    int length, int & transferred, unsigned int timeout=0) {
        check_error(libusb_interrupt_transfer(handle_, endpoint, data,
            length, &transferred, timeout));
    }
};

class Transfer : boost::noncopyable {
    static void cb(libusb_transfer *transfer) {
        try {
            Transfer & t = *reinterpret_cast<Transfer *>(transfer->user_data);
            if(t.transfer_->status != LIBUSB_TRANSFER_COMPLETED) {
                std::ostringstream msg;
                msg << "libusb transfer error " << t.transfer_->status << std::endl;
                throw Error(msg.str());
            }
            t.callback_();
        } catch(std::exception const & exc) {
            std::cout << "caught in callback: " << exc.what() << std::endl;
        } catch(...) {
            std::cout << "caught in callback: ???" << std::endl;
        }
    }
    
    libusb_transfer * transfer_;
    boost::function<void()> callback_;
public:
    Transfer(int iso_packets=0) {
        transfer_ = libusb_alloc_transfer(iso_packets);
        if(!transfer_) {
            throw Error("libusb_alloc_transfer returned NULL");
        }
    }
    ~Transfer() {
        //libusb_free_transfer(transfer_);
    }
    
    void submit() {
        check_error(libusb_submit_transfer(transfer_));
    }
    void cancel() {
        check_error(libusb_cancel_transfer(transfer_));
    }
    
    void fill_bulk(Device & dev_handle, unsigned char endpoint, unsigned char * buffer, int length, boost::function<void()> callback, unsigned int timeout=0) {
        callback_ = callback;
        libusb_fill_bulk_transfer(transfer_, dev_handle.handle_, endpoint, buffer, length, cb, reinterpret_cast<void *>(this), timeout);
    }
    void fill_interrupt(Device & dev_handle, unsigned char endpoint, unsigned char * buffer, int length, boost::function<void()> callback, unsigned int timeout=0) {
        callback_ = callback;
        libusb_fill_interrupt_transfer(transfer_, dev_handle.handle_, endpoint, buffer, length, cb, reinterpret_cast<void *>(this), timeout);
    }
};

class Context : boost::noncopyable {
    libusb_context * context_;
public:
    Context() {
        check_error(libusb_init(&context_));
    }
    ~Context() {
        libusb_exit(context_);
    }
    
    void set_debug(libusb_log_level level) {
        libusb_set_debug(context_, level);
    }
    
    std::unique_ptr<Device> open_device_with_vid_pid(uint16_t vendor_id, uint16_t product_id) {
        libusb_device_handle * h = libusb_open_device_with_vid_pid(context_,
            vendor_id, product_id);
        if(!h) {
            throw Error("open device failed");
        }
        return std::unique_ptr<Device>(new Device(h));
    }
    
    void handle_events_timeout_completed(timeval & tv, int * completed=nullptr) {
        check_error(libusb_handle_events_timeout_completed(context_, &tv, completed));
    }
};


}
