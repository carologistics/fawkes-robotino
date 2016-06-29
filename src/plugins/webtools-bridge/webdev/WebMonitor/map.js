

function Map() {

  var that = this ; 

  $(document) . ready (function()
  {   
    var $map = $("<div>  </div>") .addClass("wedgit") .addClass("container") .attr("id" , "map") ;
    $(window.$layout_container).append($map) ;//the wedgit container

  });

    // ====================================Create Viewer Scence where the map and the robots will be displaied.
    this. viewer = new ROS3D.Viewer({
      divID : 'map',
      width :  document.getElementById("map").offsetWidth ,
      height : 500 ,
      antialias : true,
      cameraPose : {
                    x : 0,
                    y : -10,
                    z : 25
                    }
    });
  



    this. visualize_marker =  function (  robot_info ) {

        //====================================== TFClinet for R1
         // Setup a client to listen to TF of R_1.
        var tfClient = new ROSLIB.TFClient({
          ros : robot_info.connection   ,
          angularThres : 0.01,
          transThres : 0.01,
          fixedFrame : '/map' ,
          rate: 1
        });

        //=======Marker arrow shape to Show robot frame /base_link of TF
        var markerClient= new ROS3D.MarkerClient({
          ros : robot_info.connection   ,
          tfClient : tfClient,
          topic : '/arrow_marker_'+ robot_info.name ,
          rootObject : that. viewer.scene
        });

    };


  this . visualize_map = function( robot_info ){
     
      var gridClient = new ROS3D.OccupancyGridClient({
        ros : robot_info .connection ,
        rootObject : that. viewer.scene
      });
  };


  this .visualize_marker_array = function( robot_info ){

     var tfClient = new ROSLIB.TFClient({
          ros : robot_info.connection   ,
          angularThres : 0.01,
          transThres : 0.01,
          fixedFrame : '/map' 
      });

      //=============================================================Navgraph Marker Array to show the 
      var markerArrayClient = new ROS3D.MarkerArrayClient({
        ros : robot_info.connection ,
        tfClient : tfClient ,
        topic : '/visualization_marker_array',
        rootObject : that. viewer.scene
      });

    };
   
}

