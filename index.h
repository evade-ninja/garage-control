PROGMEM const char index_html[] = R"(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <!-- The above 3 meta tags *must* come first in the head; any other head content must come *after* these tags -->
    <meta name="description" content="">
    <meta name="author" content="">
    <!--<link rel="icon" href="">-->

  <link rel="icon" type="image/png" href=" data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAAXNSR0IArs4c6QAAAARnQU1BAACx
jwv8YQUAAAAJcEhZcwAACxIAAAsSAdLdfvwAAAAZdEVYdFNvZnR3YXJlAHBhaW50Lm5ldCA0LjAu
MTCtCgrAAAABaElEQVQ4T4XTyytGQRzG8eNW7kIplygLG7JTUm4LZeFaFAsSkRKhV5JIIUlZWCmK
lUSkKJdYEOWS/8n3mTNToyae+rzN75x35p3fOfNGgSShHkvYtxZRh3/TjFdoUheqUYMeHOAFDQhm
BjcoNlU4ZbjDhKm89OMMKaaKo1aGMYtUXbBJwxU6TUVy8Y1sU8WpxSM2MY83NMIlD5qTqWLKctnA
A9R7K/RcKnCOY7gkMKbBNfy+P5GOJ+xgF/fQ1nXPpRJaNPrQh5d3DGLZVHHWoTehey56RmZuaIEh
LJgqzir6EFzgFkUa2OhiFnQetAtNfoba8hcox6UGcxjXwGYPF6hCL7pRgkOYCTbTmNSgAF/QL7jo
tGkHamMU2lUHXLRDvcYcU5ERHEF9uehQ6XSuIUMXbJJxggFTeVnBKfJNFU4h1J7/gH9Fx1PvegtN
KIXOfwt0JtRKO/6Mzr2+tA3tSLRgG/z/ComiH9VzQNhJA+58AAAAAElFTkSuQmCC"/>
  
    <title>GCC</title>

    <!-- Bootstrap core CSS -->
    <link rel='stylesheet' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.6/css/bootstrap.min.css' integrity='sha384-1q8mTJOASx8j1Au+a5WDVnPi2lkFfwwEAa8hDDdjZlpLegxhjVME1fgjWPGmkzs7' crossorigin='anonymous'>


    <!-- Custom styles for this template -->
    <link href="narrow.css" rel="stylesheet">
  
  </head>

  <body>

    <div class="container">
      <div class="header clearfix">
    <nav class="navbar navbar-default">
      <div class="container-fluid">
      <!-- Brand and toggle get grouped for better mobile display -->
      <div class="navbar-header">
        <button type="button" class="navbar-toggle collapsed" data-toggle="collapse" data-target="#bs-example-navbar-collapse-1" aria-expanded="false">
        <span class="sr-only">Toggle navigation</span>
        <span class="icon-bar"></span>
        <span class="icon-bar"></span>
        <span class="icon-bar"></span>
        </button>
        <a class="navbar-brand" href="#">Garage Control Center</a>
      </div>

      <!-- Collect the nav links, forms, and other content for toggling -->
      <div class="collapse navbar-collapse" id="bs-example-navbar-collapse-1">
        <ul class="nav navbar-nav navbar-right">
        <li class="active"><a href="#">Home <span class="sr-only">(current)</span></a></li>
        <li><a href="#">Alerts</a></li>
        <li><a href="#">Config</a></li>
        </ul>
      </div><!-- /.navbar-collapse -->
      </div><!-- /.container-fluid -->
    </nav>
      </div>

  <div class="jumbotron" id="moar_loading">
        <h1>More Loading</h1>
        <p class="lead">Sit tight will ya?</p>

    <div class="progress">
      <div class="progress-bar progress-bar-striped active" role="progressbar" aria-valuenow="100" aria-valuemin="0" aria-valuemax="100" style="width: 100%">
      <span class="sr-only">Loading</span>
      </div>
    </div>
      </div>
    
      <div class="jumbotron statusbox" id="d_closed" style="display: none;">
        <h1>Door is closed</h1>
        <p class="lead">Everything is just great. Door secure and all that.</p>
        <p><a class="btn btn-lg btn-success" href="#" role="button" onclick="toggleDoor();">Open Sesame</a></p>
      </div>
    
    <div class="jumbotron statusbox" id="d_opening" style="display: none;">
        <h1>Door is opening</h1>
        <p class="lead">You just asked me to open up the pod bay doors.</p>
        <p><a class="btn btn-lg btn-warning" href="#" role="button" onclick="toggleDoor();">Tap that</a></p>
      </div>
    
    <div class="jumbotron statusbox" id="d_open" style="display: none;">
        <h1>Door is open</h1>
        <p class="lead">Door is wide open yo. Hope you're home!</p>
        <p><a class="btn btn-lg btn-info" href="#" role="button" onclick="toggleDoor();">Close that</a></p>
      </div>
    
    <div class="jumbotron statusbox" id="d_closing" style="display: none;">
        <h1>Door is closing</h1>
        <p class="lead">Better get out of the way!</p>
        <p><a class="btn btn-lg btn-danger" href="#" role="button" onclick="toggleDoor();">Stop that</a></p>
      </div>
    
    <div class="jumbotron statusbox" id="d_confused" style="display: none;">
        <h1>Clueless</h1>
        <p class="lead">I'm confused! Do something!</p>
        <p><a class="btn btn-lg btn-default" href="#" role="button" onclick="toggleDoor();">I'm Feeling Lucky</a></p>
      </div>
    
    <div class="jumbotron statusbox" id="d_holdon" style="display: none;">
        <h1>Hold on to something!</h1>
        <p class="lead">The door might be moving!</p>
        <p><a class="btn btn-lg btn-danger" href="#" role="button" onclick="toggleDoor();">Emergency Stop: Never Use</a></p>
      </div>
    
    

      <div class="row marketing">
        <div class="col-lg-6">
          <h4>Subheading</h4>
          <p>Donec id elit non mi porta gravida at eget metus. Maecenas faucibus mollis interdum.</p>

          <h4>Subheading</h4>
          <p>Morbi leo risus, porta ac consectetur ac, vestibulum at eros. Cras mattis consectetur purus sit amet fermentum.</p>

          <h4>Subheading</h4>
          <p>Maecenas sed diam eget risus varius blandit sit amet non magna.</p>
        </div>

        <div class="col-lg-6">
          <h4>Subheading</h4>
          <p>Donec id elit non mi porta gravida at eget metus. Maecenas faucibus mollis interdum.</p>

          <h4>Subheading</h4>
          <p>Morbi leo risus, porta ac consectetur ac, vestibulum at eros. Cras mattis consectetur purus sit amet fermentum.</p>

          <h4>Subheading</h4>
          <p>Maecenas sed diam eget risus varius blandit sit amet non magna.</p>
        </div>
      </div>

      <footer class="footer">
        <p>2016+ evade.ninja</p>
      </footer>

    </div> <!-- /container -->

  </body>
  
  
    <!-- jQuery (necessary for Bootstrap's JavaScript plugins) -->
    <script src='https://ajax.googleapis.com/ajax/libs/jquery/1.11.3/jquery.min.js'></script>
    <!-- Include all compiled plugins (below), or include individual files as needed -->
<!-- Latest compiled and minified JavaScript -->
<script src='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.6/js/bootstrap.min.js' integrity='sha384-0mSbJDEHialfmuBBQP6A4Qrprq5OVfW37PRR3j5ELqxss1yVqOtnepnHVP9aJ7xS' crossorigin='anonymous'></script>

<script>

    function toggleDoor(callback) {
    var count;
    $.ajax({
      type: "GET",
      url: "/toggle",
      dataType: "json",
      contentType: "application/json; charset=utf-8",
      success: function (data) {                            
        if(data.success != "true"){
          alert("Toggle failed!");
        }
        
      } //success
    });
  }


  function getUpdate() {
    var count;
    $.ajax({
      type: "GET",
      url: "/status",
      dataType: "json",
      contentType: "application/json; charset=utf-8",
      success: function (data) {                            
        $(".statusbox").hide();
        $("#" + data.current_state).show(); 
        $("#moar_loading").hide();
      } //success
    });
  }

    /*
  function getUpdate(){
    $.get( "/status", function( data ) {
      $(".statusbox").hide();
      $("#" . data.current_state).show(); 
      $("#moar_loading").hide();
    });
  
  }*/

  getUpdate();

  setInterval(getUpdate, 15000);

</script>

</html>

)";


