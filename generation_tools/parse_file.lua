---------------------------------------------------------------------------
-- Written by Nikolaus Wittenstein, edited and maintained by Robert Griffin
---------------------------------------------------------------------------
--
function string:split(sep)
	-- from http://lua-users.org/wiki/SplitJoin
  local sep, fields = sep or " ", {}
  local pattern = string.format("([^%s]+)", sep)
  self:gsub(pattern, function(c) fields[#fields+1] = c end)
  return fields
end

function string:strip()
	return self:match( "^%s*(.-)%s*$" )
end

local function parse_variable(current_object, line)
	equals,_ = line:find('=')
	-- NX outputs values in this format:
	-- x,y,z = a,b,c
	-- so if there are commas, we'll step through them
	names = line:sub(1, equals-1):split(',')
	values = line:sub(equals+1):split(',')
	if #names ~= #values then
		print('Unable to parse line: '..line)
	end
	for i, name in ipairs(names) do
		value = values[i]:strip()
		value = value:gsub('\\n', '\n')
		if tonumber(value) then
			value = tonumber(value)
		end
		current_object[name:strip()] = value
	end
end

local function parse_file(filename)
	local objects = {}
	local current_object = ""
	for line in io.lines(filename) do
		if line:sub(1,1) == '*' then
			current_object = line:match("[^*]+") --strip leading asterisks
			current_object = current_object:strip()
			if not objects[current_object] then
				objects[current_object] = {}
			end
		elseif line:find('=') then
			-- Must be a variable
			parse_variable(objects[current_object], line)
		elseif #line > 0 then
			-- No "^**" or "="; must be a comment
			-- print("Skipping line: "..line)
		end
	end
	return objects
end

return parse_file
