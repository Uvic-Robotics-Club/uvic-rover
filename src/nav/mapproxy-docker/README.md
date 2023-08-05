# Google Maps Satellite + MapProxy

Run [MapProxy](https://mapproxy.org/) in a Docker container to proxy Google Maps satellite view into a WMS, TMS, or  [WMTS](https://en.wikipedia.org/wiki/Web_Map_Tile_Service) tile service. 

### `tldr:`

```
$ docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```


#### Result:


![MapProxy Demo](https://github.com/danielsnider/docker-mapproxy-googlemaps/raw/master/readme-imgs/demo.PNG)

![MapProxy Demo](https://github.com/danielsnider/docker-mapproxy-googlemaps/raw/master/readme-imgs/googlemaps.PNG)

MapProxy documentation: https://mapproxy.org/docs/nightly/tutorial.html


## Setup

1.1. Create the MapProxy configuration folder. The cached map tiles will be written to `~/mapproxy/cache_data`.

```
mkdir ~/mapproxy/
```

1.2 (Optionally) add your own `mapproxy.yaml` configuration file to override the default of proxying Google Maps satellite view over WMTS, WMS, and TMS protocols.

```
cp mapproxy.yaml ~/mapproxy/mapproxy.yaml
```

## Run 

Start the MapProxy server with `~/mapproxy` as a shared volume.

```
docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```

## Confirm 

Confirm that MapProxy is working by browsing to [http://localhost:8080/demo/](http://localhost:8080/demo/). Then if you click on "Image-format png" you will get an interactive map in your browser. 

You can also see the first map tile by browsing to [http://localhost:8080/wmts/gm_layer/gm_grid/0/0/0.png](http://localhost:8080/wmts/gm_layer/gm_grid/0/0/0.png).

## Notes

### Google Terms & Conditions 

In 2013 there was [some discussion](http://gis.stackexchange.com/questions/56982/how-to-use-mapproxy-to-serve-wms-from-reprojected-google-maps-tiles) about Google's Terms & Conditions stating that you are only allowed to access the tiles through Google's API (ie. not MapProxy).
