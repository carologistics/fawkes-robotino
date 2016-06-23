function exploration_results(){


	var destination_bridge_name = "clips" ;
	var topic_name = "exploration-result" ;

	var wedgit_id = "exploration_results";
	var $exploration_results_wedgit_div= $("<div>  </div>")	.attr('id',wedgit_id)
														 	.addClass("wedgit")
														 	.addClass("container_header_element")
														 	.append("<h2> Exploration Resutls: </h2>")
														 	.append("<p> </p>");

	$("body") . append ($exploration_results_wedgit_div);
	
	var data_to_show = {} ;

	$(document) . ready (function()
	{		

		var Robot = { name: "R 1" , ros_connection: ros}	;
		//TODO:  get this from each robot
		// for_each ( ROBOT )
		// {
			var exploration_results_listener = new ROSLIB.Topic({
			    ros : Robot.ros_connection ,
			    name : destination_bridge_name +"/" + topic_name ,
			    messageType : 'mm',
			    throttle_rate:1000,
			});
		// }

		exploration_results_listener . subscribe (function(message)
		{
			for( var index_f in message ["exploration-result"] )
			{
				var machine_f =  message ["exploration-result"] [index_f] ["machine"] ;
				var zone_f  = message ["exploration-result"] [index_f] ["zone"] ;
		
				 if( ! ( data_to_show . hasOwnProperty ( zone_f ) ) )
				 {
					data_to_show [zone_f] = Robot.name ; 
					$exploration_results_wedgit_div .find("p") .append ( "<b> "+zone_f+": </b>" + machine_f + "<sub> by:"+ Robot.name +"</sub> &nbsp &nbsp") ;
				 }			
			}
		});







		var Robot = { name: "R 2" , ros_connection: ros_2}	;
		//TODO:  get this from each robot
		// for_each ( ROBOT )
		// {
			var exploration_results_listener = new ROSLIB.Topic({
			    ros : Robot.ros_connection ,
			    name : destination_bridge_name +"/" + topic_name ,
			    messageType : 'mm',
			    throttle_rate:1000,
			});
		// }

		exploration_results_listener . subscribe (function(message)
		{
			for( var index_f in message ["exploration-result"] )
			{
				var machine_f =  message ["exploration-result"] [index_f] ["machine"] ;
				var zone_f  = message ["exploration-result"] [index_f] ["zone"] ;
		
				 if( ! ( data_to_show . hasOwnProperty ( zone_f ) ) )
				 {
					data_to_show [zone_f] = Robot.name ; 
					$exploration_results_wedgit_div .find("p") .append ( "<b> "+zone_f+": </b>" + machine_f + "<sub> by:"+ Robot.name +"</sub> &nbsp &nbsp") ;
				 }			
			}
		});






		var Robot = { name: "R 3" , ros_connection: ros_3}	;
		//TODO:  get this from each robot
		// for_each ( ROBOT )
		// {
			var exploration_results_listener = new ROSLIB.Topic({
			    ros : Robot.ros_connection ,
			    name : destination_bridge_name +"/" + topic_name ,
			    messageType : 'mm',
			    throttle_rate:1000,
			});
		// }

		exploration_results_listener . subscribe (function(message)
		{
			for( var index_f in message ["exploration-result"] )
			{
				var machine_f =  message ["exploration-result"] [index_f] ["machine"] ;
				var zone_f  = message ["exploration-result"] [index_f] ["zone"] ;
		
				 if( ! ( data_to_show . hasOwnProperty ( zone_f ) ) )
				 {
					data_to_show [zone_f] = Robot.name ; 
					$exploration_results_wedgit_div .find("p") .append ( "<b> "+zone_f+": </b>" + machine_f + "<sub> by:"+ Robot.name +"</sub> &nbsp &nbsp") ;
				 }			
			}
		});

	});

//end_Func
 }