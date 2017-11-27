function [f,h,g] = function_wrapper(obj_handle, h_handle, g_handle, x)
% Inputs:
% obj_handle - objective funtion handle obj(x)
% h_handle - equality constraints handle h(x)
% g_handle - inequality constraints handle g(x)

f = obj_handle(x);
h = h_handle(x);
g = g_handle(x);
end