# OSM-Tile Pipeline (tilemaker + tileserver)

For SolarswarmNetworks, we need to create our own. To this, we use **tilemaker** and **tileserver** to generate and serve map tiles based on **OpenStreetMap (OSM)** data.

## [Tilemaker](https://github.com/systemed/tilemaker)

**tilemaker** is a command-line tool that converts binary OpenStreetMap data (`.pbf` files) into **vector tiles** (MBTiles format).

- Reads OpenStreetMap `.pbf` files
- Applies a map style and layer definitions (partially included here; very basic tho)
- Generates vector tiles (`.mbtiles`) for offline usage

## [Tileserver](https://github.com/maptiler/tileserver-gl)

**tileserver** is just a HTTP server that serves map tiles (vector or raster) to clients such as web maps or GIS applications.

- Serves `.mbtiles` files over HTTP
- Provides tiles in standard XYZ / TileJSON formats
- Operates through files

# Typical Workflow

1. Download OpenStreetMap data (`.osm.pbf`)
2. Use **tilemaker** to generate vector tiles (`.mbtiles`)
3. Use **tileserver** to serve those tiles via HTTP
4. Display the tiles in a web or desktop map client
