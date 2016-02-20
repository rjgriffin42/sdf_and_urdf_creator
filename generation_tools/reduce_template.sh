filename="$1"
i=0
while grep -q '${' "${filename}"; do
  lua format_model.lua "${filename}" "${filename}" > nx_data${i}
  filename=nx_data${i}
  let i+=1
done

echo ${filename}
