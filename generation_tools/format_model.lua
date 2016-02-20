---------------------------------------------------------------------------
-- Written by Nikolaus Wittenstein, edited and maintained by Robert Griffin
---------------------------------------------------------------------------

local pretty = require('pl.pretty')
local parse_file = require('parse_file')

if #arg == 0 then
	print('Usage: '..arg[0]..' input_file [output_file]')
	return
end

objects = parse_file(arg[1])
-- This puts all of the objects we've just read into our global namespace.
-- We do this so that our template_replace is super duper easy.
for k, v in pairs(objects) do
	_G[k] = v
end

local function template_replace(input)
	-- input should be in the form '${expression}'
	expression = input:sub(3,#input-1)
	fn = assert(loadstring('return '..expression))
	result, retval = pcall(fn)
	if result then
		return retval
	else
		return nil
	end
end

if #arg > 1 then
	f = io.open(arg[2])
	template = f:read("*all")
	f:close()
	
	local output, numsubs = template:gsub('\${[^}]+}', template_replace)
	print(output)
else
	pretty.dump(objects)
end
