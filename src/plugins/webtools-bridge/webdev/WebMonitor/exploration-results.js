//TODO: create an object "subscription_info and pass it around and fill the ros subscription from it (will have the: distenation bridge , topic name and any other optional parameters)"
//when each object that extends them they have to set those paramaters



//Object that represents the exploration-result widget
function exploration_results( $parent )
{

	var that 							= this;
	this.	$parent_ 					= $parent;

	//to keep track of only the new data to be able to update the widget only when new (exploration-result) facts were found
	this.	data_to_show_ 				= {} ; 
	
	// subscribtion_info object paraemters
	this.	topic_name_ 				= "exploration-result" ;
	this.	destination_bridge_name_ 	= "clips" ;
	
	//coming from wedgit object parameters
 	this.	wedgit_id_ 					= "exploration_results";
	this.	$wedgit_div 				= $("<div>  </div>") 	
											.addClass("wedgit")
											.attr('id', this.wedgit_id_ )
								 			.append("<h2> Exploration Resutls: </h2>")
								 			.append("<p> </p>");
	
	this.$parent_ . append (this.$wedgit_div);


	
	//with Object oriented should be the virtual function the u have to overide t make this wedgit work ...and u shoudl give it all the parameters for a subscription.
	//u wont care how many subscriptions divs are gonna be there or who drawes them coz u can always assume there is one and only one with that name. Which is the one in this Wegit Object
	// u just know u want to display the data passed to this method the same way and put it in the same wedgit with 
	//u can even later pass the wedgit object and the subscription_info objects as a paremeter which means u can control how the wedgit looks and its parameters, from the web
	// so u can say for each wedgit how it will be fitted with in the page while u construct it ...makes usability for ur js easier and i think i might need to even wrap the ros stuff at some point to us only what we need
	//anyways...just thinking

	//Clips_facts_iterator an object that should help u navigate the facts coming as a result of a topic subscription

	this.visualize = function( robot_info ) {
			var robot_info_ 					= robot_info;

			var machine_f   ;
			var zone_f  	;

			var exploration_results_listener	= new ROSLIB.Topic( {
				ros 			: robot_info_.connection  									,
			    name 			: that.destination_bridge_name_  + "/"  + that.topic_name_  ,
			    messageType 	: 'mm' 													  	,
			    throttle_rate	: window.throttle_rate 													  	,
			});
				
			exploration_results_listener . subscribe ( 	function(message) {
				for( index_f in message ["exploration-result"] )
				{
					machine_f   =  	message ["exploration-result"] [index_f] ["machine"][0];
					zone_f  	= 	message ["exploration-result"] [index_f] ["zone"][0] ;
			
					if( ! ( that.data_to_show_ . hasOwnProperty ( zone_f ) ) )
					{
						that.data_to_show_ [zone_f] 	= 	robot_info_.name ; //keep track of the d 
						
						$(document) . ready (function() {
							that.$wedgit_div .find("p") 	.	append ( "<b> "+zone_f+": </b>" + machine_f + "<sub> by:"+ robot_info_.name +"</sub> &nbsp &nbsp") ;
						});
					}			
				}			
			
			});

	};


	// if ( $( "#myDiv" ).length ) {
 
 	// $( "#myDiv" ).show();
 
	// 

	// if( ! ($(  "#"+wedgit_id ).lenght  ) ) 
	// {
	// 	var $exploration_results_wedgit_div= $("<div>  </div>")	.attr('id',wedgit_id)
	// 														 	.addClass("wedgit")
	// 														 	.addClass("container_header_element")
	// 														 	.append("<h2> Exploration Resutls: </h2>")
	// 														 	.append("<p> </p>");
	// 	$("body") . append ($exploration_results_wedgit_div);
			
	// }



	
	// var data_to_show = {} ;

	// $(document) . ready (function()
	// {		



	

	// var Robot = { name: "R 1" , ros_connection: ros_1}	;
	// //TODO:  get this from each robot
	// // for_each ( ROBOT )
	// // {
	// 	var exploration_results_listener = new ROSLIB.Topic({
	// 	    ros : Robot.ros_connection ,
	// 	    name : destination_bridge_name +"/" + topic_name ,
	// 	    messageType : 'mm',
	// 	    throttle_rate:1000,
	// 	});
	// // }

	// exploration_results_listener . subscribe (function(message)
	// {
	// 	for( var index_f in message ["exploration-result"] )
	// 	{
	// 		var machine_f =  message ["exploration-result"] [index_f] ["machine"] ;
	// 		var zone_f  = message ["exploration-result"] [index_f] ["zone"] ;
	
	// 		 if( ! ( data_to_show . hasOwnProperty ( zone_f ) ) )
	// 		 {
	// 			data_to_show [zone_f] = Robot.name ; 
	// 			$exploration_results_wedgit_div .find("p") .append ( "<b> "+zone_f+": </b>" + machine_f + "<sub> by:"+ Robot.name +"</sub> &nbsp &nbsp") ;
	// 		 }			
	// 	}
	// });





	 // });

//end_Func
 }