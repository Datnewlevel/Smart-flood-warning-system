<!--
    This file contains the HTML and JavaScript logic for the Leaflet map.
    Node-RED will read this file and inject its content into the ui_template node.
-->
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
<style>
    #map_div { 
        height: 100%; 
        width: 100%; 
        border-radius: 10px; 
    }
</style>
<div id="map_div"></div>
<script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
<script>
(function(scope) {
    var map;
    var marker;

    // Function to initialize or update the map
    function initMap(lat, lon, color) {
        // Initialize map only once
        if (!map) {
            map = L.map('map_div').setView([lat, lon], 15);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            }).addTo(map);
        }

        // Remove old marker
        if (marker) {
            map.removeLayer(marker);
        }

        // Create a custom pulsating icon
        var iconHtml = `<div style='background-color: ${color}; width: 2rem; height: 2rem; display: block; border-radius: 3rem; transform-origin: center center;' class='pulse'></div>`;
        var customIcon = L.divIcon({
            className: 'custom-div-icon',
            html: iconHtml,
            iconSize: [30, 30],
            iconAnchor: [15, 15]
        });

        // Add new marker
        marker = L.marker([lat, lon], {icon: customIcon}).addTo(map);
    }

    // Watch for incoming messages from Node-RED
    scope.$watch('msg', function(msg) {
        if (msg && msg.payload) {
            initMap(msg.payload.lat, msg.payload.lon, msg.payload.iconColor);
        }
    });

})(scope);
</script>
