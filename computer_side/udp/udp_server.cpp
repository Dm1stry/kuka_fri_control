#include "udp_server.hpp"

using namespace server;

json server::eigenArrayToJson(const Eigen::ArrayXd& array) {
	json j = json::array(); // Создаем JSON массив
	for (int i = 0; i < array.size(); ++i) 
	{
		j.push_back(array[i]); // Добавляем элементы в массив
	}
	return j;
}

Eigen::ArrayXd server::jsonToEigenArray(const json& j) {
    Eigen::ArrayXd array(j.size()); // Создаем массив нужного размера
    for (size_t i = 0; i < j.size(); ++i) {
        array(i) = j[i]; // Заполняем элементами из JSON
    }
    return array;
}