function reachabilityToJson(reachability, path)
    regions(length(reachability)) = struct('A', [], 'b', []);
    for i = 1: length(reachability)
        r = struct('A', reshape(reachability(i).A, 1, []), 'b', reachability(i).b);
        regions(i) = r;
    end
    reachableSet = struct('regions', regions);
%     datashell = struct('reachableSet', reachableSet);
    reachableJson = jsonencode(reachableSet, 'PrettyPrint', true);
    fid = fopen(path, 'w');
    fprintf(fid, reachableJson);
    fclose(fid);
end