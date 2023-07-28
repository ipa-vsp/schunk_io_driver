#ifndef PHIDGETS_CONTAINER_ERROR_HPP_
#define PHIDGETS_CONTAINER_ERROR_HPP_

#include <cstring>
#include <stdexcept>
#include <string>

class PhidgetsContainerException : public std::exception
{
  private:
    std::string what_;

  public:
    explicit PhidgetsContainerException(const std::string &what) : what_("[Phidgets Container Exception] " + what) { }

    const char *what() const noexcept override { return what_.c_str(); }
};

#endif /* PHIDGETS_CONTAINER_ERROR_HPP_ */
