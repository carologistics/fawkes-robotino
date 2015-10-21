BOOST_INC = /usr/include
BOOST_LIB = /usr/lib

MY_CFLAGS =  -I$(BOOST_INC) -I/home/mgomaa/fawkes-robotino/src/plugins/bridge
MY_LDFLAGS = -L$(BOOST_LIB) -lboost_system 

# compile mesh classes
TARGET = Web_server
TARGET1 = web_session
TARGET2 = dispatcher

$(TARGET).o: $(TARGET).cpp
	g++ -std=c++11 $(MY_CFLAGS) $(MY_LDFLAGS) $(TARGET).cpp $(TARGET1).cpp $(TARGET2).cpp -o $(TARGET) -g
