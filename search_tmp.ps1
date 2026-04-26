Get-ChildItem 'C:\Users\panos\Desktop\spartan\data\shaders' -Recurse -Filter '*.hlsl' | ForEach-Object {
    $f = $_
    Get-Content $f.FullName | Select-String 'pass_get_transform|indirect_draw_data_out|DrawData _draw|pass_get_material_index|pass_load_draw' | ForEach-Object {
        Write-Host ($f.Name + ':' + $_.LineNumber + ': ' + $_.Line)
    }
}
