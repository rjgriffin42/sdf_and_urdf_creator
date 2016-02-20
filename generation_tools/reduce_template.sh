filename="$1"
i=0
while grep -q '${' "${filename}"; do
  lua format_model.lua "${filename}" "${filename}" > data_container${i}
  filename=data_container${i}
  let i+=1
done

echo ${filename}
