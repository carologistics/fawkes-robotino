function GameMetadata (){
	var that = this ;

	this.phase =   "N/A";
	this. game_time ="N/A";


}

window.game_info =  new GameMetadata();



function GameInfoWedgit ( $parent )  {

	var	 that 			= this ;
	this. $parent_ 		= $parent ;
	this. $wedgit_div_	= $("<div>  </div>") 	.addClass("wedgit")
												.addClass( "game_info" )
								 				// .append( $("<div > </div>" ) . addClass ("header") )
												.append( $("<div > </div>" ) . addClass ("body") )
												.append( $("<div > </div>" ) . addClass ("footer") );
	var $row ;
	var $col ;

	//=================Incoming
	$row =  $("<div> </div>" ) . addClass( "row_element" ) ;
	$col = $("<span> </span>" ) . addClass( "col_element" ) ;
	$col.append("<b> Phase: </b> ");
	var $content = $("<span> N/A </span> ")  . addClass ("phase");
	$col. append($content);
	$col.appendTo($row);

	$col = $("<span> </span>" ) . addClass( "col_element" );
	$col.append("<b> time: </b> ");
	$content = $("<span> N/A </span> ")  . addClass ("game_time");
	$col. append($content);
	$col.appendTo($row);

	$col = $("<span> </span>" ) . addClass( "col_element" ) ;
	$col.append("<b> color: </b> ");
	$content = $("<span> N/A </span> ")  . addClass ("team_color");
	$col. append($content);
	$col.appendTo($row);

	$col = $("<span> </span>" ) . addClass( "col_element" ).addClass("score") ;
	$col.append("<b> Score: </b> ");
	$content = $("<span> N/A </span> ")  . addClass ("points_cyan");
	$col. append($content);
	$content = $("<span> / </span> ")  ;
	$col. append($content);
	$content = $("<span> N/A </span> ")  . addClass ("points_magenta");
	$col. append($content);
	$col.appendTo($row);


	$row. appendTo (this.$wedgit_div_.find(".body"));


	//EndBody
	//-===================================================


	//do I need document .ready here ..Check!
	$(document) . ready (function() {
		that. $parent_ . append ( that .$wedgit_div_ ) ;
	});


	this . visualize = function () {

		that.$wedgit_div_ .find (".phase") .empty() . append (window.game_info.phase);

		that.$wedgit_div_ .find (".game_time") .empty() . append (window.game_info.game_time);

		that.$wedgit_div_ .find (".team_color") .empty() . append (window.game_info.team_color);

		that.$wedgit_div_ .find (".score .points_cyan") . empty() . append (window.game_info.points_cyan );

		that.$wedgit_div_ .find (".score .points_magenta") . empty() . append (window.game_info.points_magenta );
	}
}



function GameInfoMonitor(){

	var that 						=this;

	this. destination_bridge_name_ 	= "clips";


	//coming from wedgit object parameters
 	this.	 div_id_ 					= 	"game_info_monitor" ;
	this.	$div_ 						= 	$("<div>  </div>")	.	attr('id', this.div_id_ )
																.	addClass("game_info")
																.	 addClass("monitor");

	this .wedgit 					=  new GameInfoWedgit( this.$div_ );

	window.$layout_container. append ( this.$div_ );


	this . visualize =  function ( robot_info ){

		//---------------------------------------------------------------------Subscription to machine fact
		// Fact: (phase EXPORATION)
		var topic_name = "phase" ;
		var phase_fact_listener 	= new ROSLIB.Topic( {
			ros 			: robot_info.connection  									,
		    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
		    messageType 	: 'mm' 													  	,
		    throttle_rate	: window.throttle_rate 									  	,
		});

		phase_fact_listener . subscribe ( 	function(message) {
			var topic_name_ = "phase";

			for( index_f in message [topic_name_] )
			{
				var phase_fact = message  [topic_name_] [index_f]  ;

				if( phase_fact .fields [0] !=  that.phase ) {

					window.game_info.phase 	= phase_fact .fields [0]	;

					that . wedgit . visualize()		;

				}

			}

		});




			//---------------------------------------------------------------------Subscription to machine fact
			// Fact: (game-time 340)
			var topic_name = "game-time" ;
			var time_fact_listener 	= new ROSLIB.Topic( {
				ros 			: robot_info.connection  									,
			    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
			    messageType 	: 'mm' 													  	,
			    throttle_rate	: window.throttle_rate 									  	,
			});

			time_fact_listener . subscribe ( 	function(message) {
				var topic_name_ = "game-time";

				for( index_f in message [topic_name_] )
				{
					var time_fact = message  [topic_name_] [index_f]  ;

						window.game_info.game_time = time_fact .fields [0] / 60	;

						that . wedgit . visualize()		;
				}

			});

			// Fact: (team-color CYAN)
			var topic_name = "team-color" ;
			var team_color_fact_listener 	= new ROSLIB.Topic( {
				ros 			: robot_info.connection  									,
			    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
			    messageType 	: 'mm' 													  	,
			    throttle_rate	: window.throttle_rate 									  	,
			});

			team_color_fact_listener . subscribe ( 	function(message) {
				var topic_name_ = "team-color";

				for( index_f in message [topic_name_] )
				{
					var team_color_fact = message  [topic_name_] [index_f]  ;

					window.game_info.team_color = team_color_fact .fields [0];

					that . wedgit . visualize()		;

				}

			});

			// Fact: (team-color CYAN)
			var topic_name = "points-cyan" ;
			var points_cyan_fact_listener 	= new ROSLIB.Topic( {
				ros 			: robot_info.connection  									,
			    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
			    messageType 	: 'mm' 													  	,
			    throttle_rate	: window.throttle_rate 									  	,
			});

			points_cyan_fact_listener . subscribe ( 	function(message) {
				var topic_name_ = "points-cyan";

				for( index_f in message [topic_name_] )
				{
					var points_cyan_fact = message  [topic_name_] [index_f]  ;

					window.game_info.points_cyan =points_cyan_fact .fields [0];

					that . wedgit . visualize()		;

				}

			});

			// Fact: (team-color CYAN)
			var topic_name = "points-magenta" ;
			var points_magenta_fact_listener 	= new ROSLIB.Topic( {
				ros 			: robot_info.connection  									,
			    name 			: that.destination_bridge_name_  + "/"  + topic_name  		,
			    messageType 	: 'mm' 													  	,
			    throttle_rate	: window.throttle_rate 									  	,
			});

			points_magenta_fact_listener . subscribe ( 	function(message) {
				var topic_name_ = "points-magenta";

				for( index_f in message [topic_name_] )
				{
					var points_magenta_fact = message  [topic_name_] [index_f]  ;

					window.game_info.points_magenta =points_magenta_fact .fields [0];

					that . wedgit . visualize()		;

				}

			});



	}


}
