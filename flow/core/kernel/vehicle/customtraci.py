from flow.core.kernel.vehicle import TraCIVehicle
import numpy as np

class CustomTraCIVehicle(TraCIVehicle):
	def __init__(self,
				 master_kernel,
				 sim_params):

		TraCIVehicle.__init__(self, master_kernel, sim_params)
		self.__vehicles	= self._TraCIVehicle__vehicles

	def init_parking_params(self, veh_id, env):
		Nzones = env.network.net_params.additional_params["number_parking_zones"]

		default_params = {"desired_pzone":np.random.randint(1,Nzones-2),
					  "t_parking_elapsed":0,
					  "t_parking_base":100,
					  "t_parking_offset":0}
		default_params["current_pzone"] = default_params["desired_pzone"]
		self.__vehicles[veh_id]['parking_parameters'] = default_params

	def get_state(self, veh_id):
		return self.__vehicles[veh_id].get("state","inflow")

	def set_state(self,veh_id, state):
		 self.__vehicles[veh_id]["state"] = state

	def get_pzone(self, veh_id, env):
		if "parking_parameters" not in self.__vehicles[veh_id]:
			self.init_parking_params(veh_id, env)
		return self.__vehicles[veh_id]["parking_parameters"]["current_pzone"]

	def set_pzone(self, veh_id, pzone):
		self.__vehicles[veh_id]['parking_parameters']["current_pzone"] = pzone

	def update_tpark_elapsed(self, veh_id):
		if self.get_state(veh_id) == "parked":
			self.__vehicles[veh_id]["parking_parameters"]["t_parking_elapsed"] +=1

	def get_tparking_elapsed(self, veh_id, env):
		if "parking_parameters" not in self.__vehicles[veh_id]:
			self.init_parking_params(veh_id)
		return self.__vehicles[veh_id]["parking_parameters"]["t_parking_elapsed"]

	def get_tparking_total(self, veh_id, env):
		if "parking_parameters" not in self.__vehicles[veh_id]:
			self.init_parking_params(veh_id)
		if "t_parking_total" not in self.__vehicles[veh_id]['parking_parameters']:
			t_base = self.__vehicles[veh_id]["parking_parameters"]["t_parking_base"]
			t_offset = self.__vehicles[veh_id]["parking_parameters"]["t_parking_offset"]
			t_total = t_base + t_offset
			self.__vehicles[veh_id]["parking_parameters"]["t_parking_total"] = t_total

		return self.__vehicles[veh_id]["parking_parameters"]["t_parking_total"]

	def get_distance_to_pzone(self, veh_id, env):
		N_pz = env.network.net_params.additional_params["number_parking_zones"]
		L_total = env.network.net_params.additional_params["length_parking"] 
		L_pz = L_total/N_pz

		edge = env.k.vehicle.get_edge(veh_id)
		if edge == "inflow": x = 0
		elif edge == "outflow": x = L_pz
		elif ":" in edge: x = 0
		else: 
			Nedge = int(edge.strip("parking_"))
			x = self.get_position(veh_id)+L_pz*(Nedge-1)

		xpzone = self.get_pzone(veh_id, env)*L_pz
		l = (xpzone - x)

		return l

	def get_normalized_distance_to_pzone(self, veh_id, env):
		L_total = env.network.net_params.additional_params["length_parking"] 
		return self.get_distance_to_pzone(veh_id, env)/L

	def get_global_position(self, veh_id, env):
		x = self.get_position(veh_id)
		edge = self.get_edge(veh_id)
		N_p = env.network.net_params.additional_params["number_parking_zones"]
		L_p = env.network.net_params.additional_params["length_parking"]
		L_i = env.network.net_params.additional_params['length_inflow']

		if edge == "inflow": x_0 = 0
		elif edge == "outflow": x_0 = L_p + L_i
		elif ":" in edge: x_0 = L_i
		else: 
			Nedge = int(edge.strip("parking_"))
			x_0 = L_i + Nedge * L_P/N_P

		return x_0 + x




