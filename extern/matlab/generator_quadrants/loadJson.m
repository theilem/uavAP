function jsonObject = loadJson(jsonPath)
    fid = fopen(jsonPath);
    raw = fread(fid,inf); 
    str = char(raw'); 
    fclose(fid); 
    jsonObject = jsondecode(str);
end