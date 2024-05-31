// 172.26.108.212


/**
 * Created by mgomaa on 6/29/16.
 */



function TeamMetadata( options ){

    this . name = options.name 	|| "N/A" ;
    this . color = options.color	|| "N/A" ;
}


function MpsMetadata( options ){ //Machine data comes as the want

    this . name = options.state				|| "N/A" ;
    this . state = options.state			|| "N/A" ;
    this . team = options.team				|| "N/A" ;
    this . img = options.img				|| "N/A" ;
    this . incoming = options.incoming		|| "N/A" ;
    this . incoming_agent = options.incoming_agent || "N/A" ;
    this . produced_id = options.produced_id|| "N/A" ;
    this . loaded_id = options 	.loaded_id	|| "N/A" ;

    this . tag_found =false;
    this . explored = false;		//The exploration recognized

    var that = this ;
}

function ProductMetadata ( options ){
    // (product (id 73991192) (product-id 0) (rings) (cap BLACK) (base RED) (sync-id 51))
    this.id;
    this. product-id;
    this.rings;
    this.cap;
    this.base;

}
