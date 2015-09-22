# location of the Python header files
 
PYTHON_VERSION = 2.7
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)
 
# location of the Boost Python include files and library
 
BOOST_INC = /usr/include
BOOST_LIB = /usr/lib

CFLAGS = -I$(PYTHON_INCLUDE) -I$(BOOST_INC)
 
# compile mesh classes
TARGET = bridge_interface
 


depend: .depend

.depend: $(TARGET).cpp
	rm -f ./.depend
	g++ $(CFLAGS) -MM $^ > ./.depend;

include .depend


$(TARGET).so: $(TARGET).o
	g++ -shared -Wl,--export-dynamic $(TARGET).o -L$(BOOST_LIB) -lboost_python -L/usr/lib/python$(PYTHON_VERSION)/config -lpython$(PYTHON_VERSION) -o $(TARGET).so
 
$(TARGET).o: $(TARGET).cpp
	g++ $(CFLAGS) -fPIC -c $(TARGET).cpp

