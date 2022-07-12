
function refresh_wp_info()
    local index = mission:get_current_nav_index()
  
    if index == 3 then
        sw1 = mission:get_item(2)
        sw1:param1(3)
        sw1:param3(30)
        sw1:param4(50)
        mission:set_item(2, sw1)
    end 
end

function update()
    refresh_wp_info()
    return update, 1000 -- 1Hz
end
  
return update(), 1000 -- start with a 1 sec delay