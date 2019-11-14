#include <boost/test/unit_test.hpp>
#include <spartan/Dummy.hpp>

using namespace spartan;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    spartan::DummyClass dummy;
    dummy.welcome();
}
