var ros ;
function init() {
  // Connect to ROS.
  ros = new ROSLIB.Ros({
    url : 'ws://localhost:6060'
  });
}

// function products() {

// var div_products = "<div id='products'> </div>";
// $("body").append(div_products);
  
// var products_listener = new ROSLIB.Topic({
//   ros : ros,
//   name : 'clips/product',
//   messageType : 'mm',
//   throttle_rate:1000
// });


// products_listener.subscribe(function(message) {


// $("#products").empty();

// for (var i = Things.length - 1; i >= 0; i--) {
//   Things[i]
// };


// });

// }


