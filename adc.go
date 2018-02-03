package adc

// #cgo LDFLAGS: -lbcm2835
// #include "ADS1256.h"
import "C"
import "fmt"

func Open() error {
	rv := C.ADS1256_init()

	if rv != 0 {
		return fmt.Errorf("Could not initialize ADS1256. rv = ", rv)
	}

	return nil
}

func Close() {
	C.ADS1256_dest()
}

func Sample(scope uint8, channel uint8) int32 {
	var cscope = C.uint8_t(scope)
	var cchannel = C.uint8_t(channel)
	var data = int32(C.ADS1256_Collect(cscope, cchannel))
	return data
}
