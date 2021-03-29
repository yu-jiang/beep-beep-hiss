classdef satelliteMapParser < handle

    properties (SetAccess = private)
        zmlvl = 0;
        
        tile_width = 1;
        tile_height = 1;
        
        tile_size = 256;
        
        tilex = 0;
        tiley = 0;
        
        pixelx = 0;
        pixely = 0;
        
        image = [];
        local_cached_map = [];
    end
    
    methods
        function this = satelliteMapParser(tile_x, tile_y, tile_w, tile_h, zoomlvl)
            
            this.zmlvl = zoomlvl;
            this.tilex = tile_x;
            this.tiley = tile_y;
            this.tile_width = tile_w;
            this.tile_height = tile_h;
            
            assert(rem(tile_w,2) == 1, "tile width must be an odd number");
            assert(rem(tile_h,2) == 1, "tile height must be an odd number");
            
            current_folder = fileparts( mfilename('fullpath'));
            local_map_folder = fullfile(current_folder, 'localmap');
            if exist(local_map_folder, 'dir') ~= 7
                mkdir(local_map_folder);
            end
            local_map_name = fullfile(local_map_folder, 'localmaps.mat');
            
            try
                d = load(local_map_name);
                this.local_cached_map = d.local_cached_map;
            catch Ex
                disp(Ex.message)
                this.local_cached_map = containers.Map;
            end
            
            fill_image(this);
            
            local_cached_map = this.local_cached_map;
            save(local_map_name, 'local_cached_map');
        end
        
        
        function [wx,wy] = getPathXY(this, path_lat, path_lng)
            % Compute world xy
            [wx, wy] = satelliteMapParser.getWorldXY(path_lat, path_lng, this.zmlvl, this.tile_size);
            % Compute local xy
            wx = wx - this.tile_size * (this.tilex - floor((this.tile_width - 1)/2));
            wy = wy - this.tile_size * (this.tiley - floor((this.tile_height - 1)/2));
        end
        
        function [path_lat, path_lng] = getPathCoord(this, wx, wy)
            % Compute world xy
            wx = wx + this.tile_size * (this.tilex - floor((this.tile_width - 1)/2));
            wy = wy + this.tile_size * (this.tiley - floor((this.tile_height - 1)/2));
            % Compute coord
            [path_lat, path_lng] = satelliteMapParser.getLatLon(wx, wy,  this.zmlvl, this.tile_size);
        end
        
        function show(this)
            image(this.image);
            shading flat
            axis equal
        end
    end
    
    methods (Access = private)
        function fill_image(this)
            image_width = this.tile_size * this.tile_width;
            image_height = this.tile_size * this.tile_height;
            this.image = uint8(ones(image_height, image_width, 3));
            
            for ii = 1:this.tile_width
                for jj = 1:this.tile_height
                    fprintf("reading row (%d / %d) and ", jj, this.tile_height)
                    fprintf("column (%d / %d) \n", ii, this.tile_width)
                    im = this.getMapBoxTile(ii - (this.tile_width+1)/2 + this.tilex, ...
                        jj - (this.tile_height+1)/2 + this.tiley);
                    this.image((jj-1) * this.tile_size + (1:this.tile_size), ...
                        (ii-1) * this.tile_size + (1:this.tile_size), :) = im;
                end
            end
        end
        
        function im = getMapBoxTile(this, tilex, tiley)
            
            zoomlvl = this.zmlvl;
            % construct key
            key = [num2str(tilex), 'x', num2str(tiley), '_', num2str(zoomlvl)];
            if isKey(this.local_cached_map, key)
                fprintf("loading #%s from cache\n", key)
                im = this.local_cached_map(key);
            else
                fprintf("loading #%s from web\n", key)
                urlsk = "https://api.mapbox.com/v4/mapbox.satellite/%d/%d/%d.png?access_token=";
                mpurl = sprintf(urlsk, floor(zoomlvl), floor(tilex), floor(tiley));
                url = mpurl + getMapBoxToken;
                im = webread(url, 'Timeout', 30);
                this.local_cached_map(key) = im;
            end
        end
    end
    
    % Static methods
    methods (Static)
        % Create map from center gps coord, zoom leve, and number of tiles
        function map = createMap(lat, lng, zmlvl, tilew, tileh)
            assert(isscalar(lat), 'Latitude must be scalar');
            assert(isscalar(lng), 'Longitude must be scalar');
            
            [tilex, tiley] = satelliteMapParser.getTileXY(lat, lng, zmlvl);
            map = satelliteMapParser(tilex, tiley, tilew, tileh, zmlvl);
        end
        
        function [map, info] = createMapFromPath(lat, lng, maxtiles)
            if nargin <3
                maxtiles = 121;
            end
            
            % compute the center tile coorindate
            zmlvl = 23;
            tilew = Inf;
            tileh = Inf;
            while tilew*tileh > maxtiles
                zmlvl = zmlvl - 1;
                [x, y] = satelliteMapParser.getWorldXY(lat, lng, zmlvl, 1);
                
                tilex0 = floor(min(x));
                tiley0 = floor(min(y));
                tilex1 = floor(max(x));
                tiley1 = floor(max(y));
                
                tilex = floor(0.5*tilex0 + 0.5*tilex1);
                tiley = floor(0.5*tiley0 + 0.5*tiley1);
                
                tilew = 1 + 2 * max((tilex - tilex0), (tilex1 - tilex));
                tileh = 1 + 2 * max((tiley - tiley0), (tiley1 - tiley));
            end
            
            map = satelliteMapParser(tilex, tiley, tilew, tileh, zmlvl);
            info.zmlvl = zmlvl;
        end
        
        function [tilex, tiley] = getTileXY(lat, lng, zmlvl)
            [worldx, worldy] = satelliteMapParser.getWorldXY(lat, lng, zmlvl, 1);
            tilex = floor(worldx);
            tiley = floor(worldy);
        end
       
        function [wx,wy]  = getWorldXY(lat, lng, zmlvl, tile_size)
            siny = sin(deg2rad(lat));
            siny = max(min(siny, 0.9999), -0.9999);
            wx = tile_size * (2^zmlvl) * (0.5 + lng ./ 360.0);
            wy = tile_size * (2^zmlvl) * (0.5 - log((1 + siny) ./ (1 - siny)) ./ (4 * pi));
        end
        
        function [lat, lng] = getLatLon(wx, wy, zmlvl, tile_size)
            % Compute world coordinate
            lng  =  (wx ./ tile_size ./ (2^zmlvl) - 0.5) .* 360;
            hsiny = exp( 4 * pi * (0.5 - wy ./ (tile_size  .* (2^zmlvl))));
            siny = (hsiny - 1) ./ (hsiny + 1);
            lat = asin(siny);
            lat = rad2deg(lat);
        end
    end
end

%% There has to be a local function file in the following format
